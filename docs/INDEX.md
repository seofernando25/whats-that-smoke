# INDEX

`purpose:=minimal-token context; audience:=LLM; prose:=min; truth:=typed`

## load

```text
L0 README.md
L1 STATE.md + SAFETY.md
L2 HARDWARE.md + NETWORK.md + COMMANDS.md + DASHBOARD.md
L3 PROTOCOL.md
```

## epistemics

```text
✓ observed/tested
~ inferred
? unknown/unverified
× absent/disabled/fail
! hazard/blocker
∆ changed
@ locator
# reference
```

`conflict := newest-observation > inference > vendor-doc > assumption`

`mutation := declare intent -> safety gate -> smallest effect -> verify -> STATE∆`

`never-store := passwords|tokens|private-keys|router-secrets`

`ARUCO.md := 50mm IDs0..4 + pose/follow equations + autonomous failsafes`
