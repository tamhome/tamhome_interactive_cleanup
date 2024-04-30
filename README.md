# Interactive Cleanup

## How to launch

- Terminal 1

```bash
cd usr/tamhome_ws
singularity shell -B /run/user/1000,/var/lib/dbus/machine-id --nv env/sandbox_sigverse/
source /entrypoint.sh
source devel/setup.bash
roslaunch sigverse_hsrb_utils sigverse_ros_bridge.launch
```

- Terminal 2

```bash
singularity shell -B /run/user/1000,/var/lib/dbus/machine-id --nv env/sandbox_sigverse/
source /entrypoint.sh
source devel/setup.bash
roslaunch sigverse_hsrb_utils bring_up.launch
```

- Terminal 3
  - ローカルでの実行

```bash
source devel/setup.bash
roslaunch tam_object_detection hsr_head_rgbd_lang_sam_service.launch
```

- Terminal 4
  - Terminal 3のコマンドにて，langsamサーバを起動させてから実行する
  - 最後のコマンドに `wait_to_ready:=false` が**含まれていない**ことを確認

```bash
singularity shell -B /run/user/1000,/var/lib/dbus/machine-id --nv env/sandbox_sigverse/
source /entrypoint.sh
source devel/setup.bash
roslaunch tamhome_interactive_cleanup interactive_cleanup.launch
```
