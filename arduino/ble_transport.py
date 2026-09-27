"""Bounded, ordered BLE delivery with cumulative acknowledgements."""
import asyncio,struct,time,os
from bleak import BleakClient,BleakScanner
SERVICE='cfb00001-7c42-4cb8-9ac0-27d96c562e11'
DATA='cfb00002-7c42-4cb8-9ac0-27d96c562e11'
ACK='cfb00003-7c42-4cb8-9ac0-27d96c562e11'
async def stream(on_sample, on_connect, stats, duration=None, inject_drop_every=0, ack_pause=None):
 device=await BleakScanner.find_device_by_filter(lambda d,a:SERVICE in [u.lower() for u in a.service_uuids],timeout=15)
 if device is None:raise RuntimeError('Bluetooth board not found. Check power.')
 async with BleakClient(device,timeout=20) as client:
  on_connect()
  stats.update(received=0,delivered=0,duplicates=0,injected_drops=0,unrecoverable=0,recovered=0)
  expected=None;last_ack=None;pending={};missing=set();changed=asyncio.Event();last_received=time.monotonic();started=last_received
  def receive(_,data):
   nonlocal expected,last_ack,last_received
   last_received=time.monotonic()
   if len(data)!=20:return
   stamp,*v=struct.unpack('<I6hI',data);seq=v.pop();stats['received']+=1
   if expected is None:expected=seq
   if inject_drop_every and stats['received']%inject_drop_every==0:
    stats['injected_drops']+=1;return
   distance=(seq-expected)%2**32
   if distance>=2**31:
    stats['duplicates']+=1;changed.set();return
   # Beyond firmware retention: do not wait forever for evicted samples.
   if distance>=256:
    stats['unrecoverable']+=distance;pending.clear();missing.clear();expected=seq
   elif distance:
    missing.add(expected)
   pending[seq]=(stamp,v)
   while expected in pending:
    stamp,v=pending.pop(expected)
    if expected in missing:stats['recovered']+=1;missing.discard(expected)
    on_sample([stamp,*[x/8192 for x in v[:3]],*[x/16.384 for x in v[3:]]])
    stats['delivered']+=1;last_ack=expected;expected=(expected+1)%2**32
   changed.set()
  await client.start_notify(DATA,receive)
  while client.is_connected:
   now=time.monotonic()
   if duration and now-started>=duration:break
   if now-last_received>5:raise RuntimeError('BLE stream stalled; reconnecting…')
   if last_ack is not None and changed.is_set() and not (ack_pause and ack_pause[0]<=now-started<ack_pause[1]):
    changed.clear();await asyncio.wait_for(client.write_gatt_char(ACK,struct.pack('<I',last_ack),response=False),3)
   await asyncio.sleep(.02)
