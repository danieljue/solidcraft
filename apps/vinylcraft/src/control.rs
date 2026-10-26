//! Local control socket for VinylCraft.
//!
//! The GUI exposes a loopback JSON-RPC endpoint so a separate process (the CLI, a script, an
//! MCP client) can ask a running window for its state or invoke a tool. The protocol is the
//! same newline-delimited JSON-RPC the stdin server speaks, so one client parser serves both
//! transports.

use serde_json::{Value, json};
use std::io::{BufRead, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use eframe::egui;

use crate::jobs::CtrlMsg;
use crate::tools::{rpc_error, vinyl_tools};

/// Set when a control client asks the app to close. A static is the simplest handoff between
/// the socket thread and the UI thread: the GUI polls it each frame and sends the close
/// command, so nothing has to be threaded through the app's own state.
pub(crate) static QUIT_REQUESTED: AtomicBool = AtomicBool::new(false);

pub(crate) const CONTROL_PORT_DEFAULT: u16 = 47777;

/// Where the GUI advertises the port it actually bound. The default port is the common case,
/// but a second instance or a taken port pushes the listener to an ephemeral one; without a
/// hint file a client would have no way to find it.
pub(crate) fn control_port_file() -> std::path::PathBuf {
    std::env::temp_dir().join("vinylcraft.ctl")
}

/// Serve control requests on 127.0.0.1. Binds the default port when free, otherwise an
/// ephemeral one; the port is reported in the app log so a client can find it.
pub(crate) fn spawn_control_server(tx: mpsc::Sender<CtrlMsg>, ctx: egui::Context) -> std::io::Result<u16> {
    let listener = TcpListener::bind(("127.0.0.1", CONTROL_PORT_DEFAULT))
        .or_else(|_| TcpListener::bind(("127.0.0.1", 0)))?;
    let port = listener.local_addr()?.port();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let tx = tx.clone();
            let ctx = ctx.clone();
            std::thread::spawn(move || {
                let _ = handle_control_client(stream, tx, ctx);
            });
        }
    });
    Ok(port)
}

/// One client connection: newline-delimited JSON-RPC, the same shape as the stdin server so a
/// client can speak either transport without changing its parser.
pub(crate) fn handle_control_client(
    stream: TcpStream,
    tx: mpsc::Sender<CtrlMsg>,
    ctx: egui::Context,
) -> std::io::Result<()> {
    let mut writer = stream.try_clone()?;
    let mut reader = std::io::BufReader::new(stream);
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            break;
        }
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let msg: Value = match serde_json::from_str(trimmed) {
            Ok(v) => v,
            Err(e) => {
                writeln!(writer, "{}", rpc_error(Value::Null, -32700, format!("parse error: {e}")))?;
                writer.flush()?;
                continue;
            }
        };
        let id = msg.get("id").cloned().unwrap_or(Value::Null);
        let method = msg.get("method").and_then(Value::as_str).unwrap_or("");
        let params = msg.get("params").cloned().unwrap_or(Value::Null);
        let reply: Result<Value, (i64, String)> = match method {
            "initialize" => Ok(json!({
                "protocolVersion": "2025-06-18",
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "vinylcraft-gui", "version": env!("CARGO_PKG_VERSION")}
            })),
            "ping" => Ok(json!({})),
            // Closing is a flag flip plus a repaint request: the UI thread owns the window and
            // is the only place allowed to send the close command.
            "quit" => {
                QUIT_REQUESTED.store(true, Ordering::SeqCst);
                ctx.request_repaint();
                Ok(json!({"ok": true, "message": "closing"}))
            }
            "state" => {
                let (rtx, rrx) = mpsc::channel();
                let _ = tx.send(CtrlMsg::State(rtx));
                ctx.request_repaint();
                match rrx.recv_timeout(Duration::from_secs(5)) {
                    Ok(v) => Ok(v),
                    Err(_) => Err((-32603, "app did not respond".into())),
                }
            }
            "tools/list" => Ok(json!({"tools": vinyl_tools()})),
            "tools/call" => {
                let name = params.get("name").and_then(Value::as_str).unwrap_or("").to_string();
                let arguments = params.get("arguments").cloned().unwrap_or(json!({}));
                let (rtx, rrx) = mpsc::channel();
                let _ = tx.send(CtrlMsg::Call {
                    name: name.clone(),
                    args: arguments,
                    reply: rtx,
                });
                ctx.request_repaint();
                match rrx.recv_timeout(Duration::from_secs(30)) {
                    Ok(Ok(v)) => Ok(json!({"content": [{"type": "text", "text": v.to_string()}]})),
                    Ok(Err(e)) => Ok(json!({
                        "content": [{"type": "text", "text": format!("error: {e}")}],
                        "isError": true
                    })),
                    Err(_) => Err((-32603, format!("tool `{name}` timed out"))),
                }
            }
            other => Err((-32601, format!("unknown method `{other}`"))),
        };
        match reply {
            Ok(result) => writeln!(writer, "{}", json!({"jsonrpc": "2.0", "id": id, "result": result}))?,
            Err((code, message)) => writeln!(writer, "{}", rpc_error(id, code, message))?,
        }
        writer.flush()?;
    }
    Ok(())
}

/// Send one request to a running GUI's control socket and return its reply.
pub(crate) fn control_request(port: u16, req: Value) -> Result<Value, String> {
    let addr: std::net::SocketAddr = ([127, 0, 0, 1], port).into();
    let mut stream = TcpStream::connect_timeout(&addr, Duration::from_secs(2))
        .map_err(|e| format!("connect 127.0.0.1:{port}: {e}"))?;
    stream.set_read_timeout(Some(Duration::from_secs(10))).ok();
    stream.set_write_timeout(Some(Duration::from_secs(2))).ok();
    let mut line = req.to_string();
    line.push('\n');
    stream.write_all(line.as_bytes()).map_err(|e| e.to_string())?;
    stream.flush().ok();
    let mut reader = std::io::BufReader::new(stream);
    let mut reply = String::new();
    reader.read_line(&mut reply).map_err(|e| e.to_string())?;
    serde_json::from_str(reply.trim()).map_err(|e| format!("bad reply: {e}"))
}

/// Last resort when no control socket answers: kill by PID, and only after confirming the
/// process's ExecutablePath is *this* binary. This cannot flush state — it is the blunt path —
/// so it only runs after the graceful request fails.
///
/// Image-name matching is case-insensitive on Windows. This binary's former image name
/// collided with an unrelated program of the same name installed on this machine, so
/// `/IM <name>` would have killed that one too. It is `vinylcraft.exe` now. Even so,
/// identity is the executable path, never the image name.
pub(crate) fn quit_by_process_name() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(windows)]
    {
        let me = std::env::current_exe()
            .ok()
            .and_then(|p| p.canonicalize().ok());
        // tasklist cannot show a process's path; WMI can. Ask for both id and path so each
        // candidate is checked before anything is terminated.
        let query = "Get-CimInstance Win32_Process -Filter \"Name='vinylcraft.exe'\" | \
                     ForEach-Object { $_.ProcessId.ToString() + \"`t\" + $_.ExecutablePath }";
        let out = std::process::Command::new("powershell")
            .args(["-NoProfile", "-Command", query])
            .output()
            .map_err(|e| format!("powershell: {e}"))?;
        let text = String::from_utf8_lossy(&out.stdout);
        let mut killed = 0usize;
        for line in text.lines() {
            let Some((pid, path)) = line.trim().split_once('\t') else {
                continue;
            };
            let Ok(pid) = pid.trim().parse::<u32>() else {
                continue;
            };
            // The process running `--quit` is itself a vinylcraft.exe from this path; never
            // target it.
            if pid == std::process::id() {
                continue;
            }
            let same = match (&me, path.trim()) {
                (Some(m), p) if !p.is_empty() => std::path::Path::new(p)
                    .canonicalize()
                    .map(|q| &q == m)
                    .unwrap_or(false),
                _ => false,
            };
            if !same {
                eprintln!("skipping PID {pid} ({path}): not this executable");
                continue;
            }
            let status = std::process::Command::new("taskkill")
                .args(["/PID", &pid.to_string(), "/F"])
                .status()
                .map_err(|e| format!("taskkill: {e}"))?;
            if status.success() {
                println!("terminated vinylcraft.exe (PID {pid})");
                killed += 1;
            } else {
                return Err(format!("taskkill exited with {status}").into());
            }
        }
        if killed == 0 {
            return Err(
                "no running vinylcraft.exe matched this executable's path; close the window manually"
                    .into(),
            );
        }
        Ok(())
    }
    #[cfg(not(windows))]
    {
        // On Unix, match on this executable's full path rather than a bare name that other
        // software may also use.
        let me = std::env::current_exe()
            .ok()
            .and_then(|p| p.to_str().map(str::to_owned))
            .unwrap_or_else(|| "vinylcraft".into());
        let status = std::process::Command::new("pkill")
            .args(["-f", &me])
            .status()
            .map_err(|e| format!("pkill: {e}"))?;
        if status.success() {
            println!("terminated {me}");
            Ok(())
        } else {
            Err(format!("pkill exited with {status}").into())
        }
    }
}
