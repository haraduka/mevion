#!/bin/bash

# 一時名のprefix
temp_prefix="can_temp_"

# それぞれの can_usb_* がどの実デバイスにリンクしてるか取得
get_iface() {
  dev_path=$(udevadm info -q path -n /dev/can_usb_$1 2>/dev/null)
  [ -n "$dev_path" ] && find /sys/class/net -lname "*$dev_path*" -exec basename {} \; 2>/dev/null
}

# 存在する can_usb_* を列挙
can_ifaces=()
idx=0
while [ -e /dev/can_usb_$idx ]; do
  iface=$(get_iface $idx)
  if [ -n "$iface" ]; then
    can_ifaces+=("$iface")
  fi
  idx=$((idx+1))
done

if [ ${#can_ifaces[@]} -eq 0 ]; then
  echo "No CAN USB devices found."
  exit 1
fi

# すでに canX が存在する場合は temp に避ける
for i in "${!can_ifaces[@]}"; do
  iface=${can_ifaces[$i]}
  sudo ip link set "$iface" down || true
  if [[ "$iface" =~ ^can[0-9]+$ ]]; then
    sudo ip link set "$iface" name "${temp_prefix}${i}"
    can_ifaces[$i]="${temp_prefix}${i}"
  fi
done

# 順番に can0, can1, ... にリネーム
for i in "${!can_ifaces[@]}"; do
  sudo ip link set "${can_ifaces[$i]}" name "can$i"
done

# CANセットアップ
for i in "${!can_ifaces[@]}"; do
  sudo ip link set "can$i" type can bitrate 1000000
  sudo ip link set "can$i" up
done
