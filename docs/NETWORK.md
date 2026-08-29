# Network

`FillMeUpLink` → DHCP; expected `wlan0=192.168.8.170/24`.

NM profile hardening:

- `autoconnect=yes`; priority `100`; retries `0` = forever.
- Wi-Fi power save `disabled` → fewer idle/link-drop failures.
- route metric: Wi-Fi `600`; Ethernet `100` → Ethernet preferred, Wi-Fi remains fallback.
- observed boot association+DHCP ≈8s.

Dashboard safety is independent of network reconnect: browser sends fresh drive intent every `150ms`; hardware guard brakes after `600ms` without a fresh drive message. Generic WebSocket heartbeat never renews motion.
