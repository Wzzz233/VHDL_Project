# BGP Live Web Control

This is a lightweight phone/browser control panel for `pplcnet_bgp_live`.

## Files

- `server.py` - Python standard-library HTTP server.
- `static/` - browser UI.

The server does not read `/dev/fpga_dma0`. It starts/stops `pplcnet_bgp_live` and serves files written by the live process:

- `/tmp/bgp_web/latest.bmp`
- `/tmp/bgp_web/results.json`
- `/tmp/bgp_web/status.json`
- `/tmp/bgp_web/live.log`

## Board Run

Copy `pplcnet_bgp_live` and `web_control/` to the same directory on the board, then run:

```bash
cd ~/ARM
python3 web_control/server.py --host 0.0.0.0 --port 8080 --arm-dir ~/ARM --preview-dir /tmp/bgp_web --log-path /tmp/bgp_web/live.log
```

Open from a phone on the same Wi-Fi network:

```text
http://<board-ip>:8080
```

If the live binary needs root and passwordless sudo is configured, add `--sudo` to the server command. Otherwise start `pplcnet_bgp_live` manually with:

```bash
sudo ./pplcnet_bgp_live ... --display-every 2 --web-preview-dir /tmp/bgp_web --web-preview-fps 5
```

Then keep the web server running without using its Start/Stop buttons.

## Notes

- Default stable display mode is `display_every=2`.
- Preview uses BMP to avoid extra JPEG/ffmpeg dependencies.
- Preview output is downscaled to 640px width.
