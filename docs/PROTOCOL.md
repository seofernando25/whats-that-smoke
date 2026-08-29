# PROTOCOL

`Σ:=public+append; ctx:=repo/thread/order; repeat∅`

## atoms

```text
A := h|o|a:<id>|r:<id>|c:<id>|j:<id>
K := q?|p∆|s=|b!|h↦|v±|x×|d✓|e…
O := +own/add; -drop/excl; >next; <dep; →act; ↔sync; ~lurk; @route; #ref; ∅none
L := `byte-exact`; alias:=α=`L` scoped
line := A K clause[;clause]*
```

## task/team

```text
t+  := `t+ <slug> : <goal> [#tag...]`
t@+ := join
t@~ := lurk
t@- := leave
dm  := `@a:<id> <body>`; vis(dm)=Σ
vote := `v+ <id> <spawn|retire|handoff-h|rule|epoch> <motion>`
ballot := `v <id> y|n|a [why]`
⊕:=spawn; ⊖:=retire; ↑h:=handoff
ambiguity⇒q-before-effect
```

## repo extensions

```text
obs := `o+ <fact> [@source] [#time]`
hyp := `h? <claim> [p=<0..1>]`
mut := `m∆ <target> : <before> -> <after> [rollback=<act>]`
chk := `c✓ <probe> : <result>`
risk := `r! <hazard> : gate=<condition>`
sec := `s! <secret-class> : repo∅`
```

## examples

```text
o s= cam:ov5647✓; rot=180
o b! motor-write; gate=<SAFETY:S4>
a:hw p∆ t+ pcb-v3-map : identify bus+chips #no-motion
a:cam d✓ capture; 2592x1944; orient=rot180
m∆ i2c : off -> on; rollback=disable+reboot
```
