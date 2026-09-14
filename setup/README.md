# setup/ — ติดตั้งและเปิดระบบ ด้วยคลิกเดียว

Start with the [illustrated step-by-step guide](../docs/step_by_step.md) for screenshots of each UI tab and the [Wi-Fi setup walkthrough](../docs/wifi_setup.md).

## เครื่องหุ่นยนต์ (Ubuntu) — ใช้ไฟล์นี้ไฟล์เดียว

```bash
cd setup
chmod +x setup_and_run.sh          # ครั้งแรกครั้งเดียว
./setup_and_run.sh
```

หรือทำเป็นไอคอนบนหน้าจอ แล้วดับเบิลคลิกได้เลย:

```bash
./install_shortcut.sh
```

สคริปต์จะทำให้ตามลำดับ:

| ขั้น | ทำอะไร |
|---|---|
| 1 | ตรวจเครื่อง: Ubuntu รุ่นอะไร, python3, เจอโปรเจกต์ไหม |
| 2 | ตรวจ **ROS 2** ถ้าไม่มีจะติดตั้ง `ros-humble-ros-base` ให้ (ถามก่อน) ถ้ามีรุ่นอื่นอยู่จะถามว่าใช้รุ่นนั้นไหม |
| 3 | ติดตั้งของที่ขาด: `python3-yaml`, `colcon`, `git`, **micro-ROS agent** (ถ้า apt ไม่มีจะ build จากซอร์สให้) |
| 4 | `colcon build --symlink-install` |
| 5 | ตรวจ env: `ROS_DOMAIN_ID=10`, พอร์ต 8080 ว่างไหม, เปิดไฟร์วอลล์ให้ (8080/tcp + 8888/udp) |
| 6 | **เปิดระบบทั้งหมด** แล้วบอกที่อยู่เว็บ เช่น `http://192.168.1.50:8080` |

**รันครั้งที่สองเป็นต้นไป** ถ้าติดตั้งครบแล้ว จะข้ามไปขั้นที่ 6 ทันที คือเปิดเว็บให้คุมหุ่นเลย

### ตัวเลือกเพิ่ม

| คำสั่ง | ทำอะไร |
|---|---|
| `./setup_and_run.sh --check` | ตรวจอย่างเดียว ไม่ติดตั้ง ไม่เปิดระบบ |
| `./setup_and_run.sh --yes` | ไม่ต้องถาม ตอบใช่ทั้งหมด (ติดตั้งอัตโนมัติ) |
| `./setup_and_run.sh --service` | ติดตั้งให้เปิดเองทุกครั้งที่บูตเครื่อง (systemd) |
| `./setup_and_run.sh --firmware` | ลง PlatformIO ด้วย สำหรับแฟลชบอร์ด ESP32 |
| `./setup_and_run.sh --reinstall` | ทำใหม่ทุกขั้นตอน |
| `./setup_and_run.sh --no-browser` | ไม่ต้องเปิดเบราว์เซอร์ให้ |

บันทึกการติดตั้งอยู่ที่ `~/.gps_localize/setup.log`
ไฟล์ตั้งค่าอยู่ที่ `~/.gps_localize/*.yaml`

## เครื่อง Windows

ดับเบิลคลิก **`setup_and_run.bat`** จะถามว่า

* **พิมพ์ไอพีของหุ่นยนต์** → เปิดหน้าเว็บของหุ่นจริง → **ควบคุมได้เต็มที่**
* **กด Enter เฉย ๆ** → เปิดโปรแกรมดูอย่างเดียวในเครื่อง (ควบคุมหุ่นไม่ได้)

ถ้ายังไม่มีไฟล์ `.exe` และเครื่องมี Python อยู่ จะถามว่าจะสร้างให้ไหม
Windows สามารถรันเซิร์ฟเวอร์หุ่นจริงผ่าน Docker ได้: Wi-Fi ใช้ Docker Desktop; USB ใช้ WSL2 + Docker Engine ภายใน Ubuntu ดู [คู่มือ Docker](../docs/docker.md)

## ลำดับการใช้งานจริงหน้างาน

```
เครื่อง Ubuntu (server)          มือถือ / แท็บเล็ต / คอมอื่น
  ดับเบิลคลิก GPS_Localize   ──►   เปิดเบราว์เซอร์ http://<ไอพี>:8080
  (หรือ ./setup_and_run.sh)        ควบคุมหุ่นได้ทุกเครื่องใน Wi-Fi เดียวกัน
```

## แก้ปัญหา

| อาการ | ทำอะไร |
|---|---|
| สคริปต์ขึ้น `sudo` ขอรหัสผ่าน | ปกติ ใส่รหัสผ่านผู้ใช้ Ubuntu (ใช้ตอนติดตั้งเท่านั้น) |
| ติดตั้ง ROS ค้าง/ช้า | ใช้เน็ตช้า ประมาณ 1 GB รอจนจบ หรือรันใหม่ได้ ทำต่อจากเดิม |
| `port 8080 is already in use` | มีโปรแกรมเปิดอยู่แล้ว: `sudo systemctl stop gps_localize` หรือปิดหน้าต่างเดิม |
| มือถือเปิดเว็บไม่ได้ | ต้องอยู่ Wi-Fi เดียวกัน และไฟร์วอลล์ต้องเปิด 8080/tcp (สคริปต์ถามให้ตอนติดตั้ง) |
| หน้าเว็บขึ้นแต่ค่าว่างหมด | บอร์ด ESP32 ยังไม่ต่อ ดู `ros2 topic hz /gps_localize/telemetry` และ [../docs/install.md](../docs/install.md) |

## Docker and Windows USB

For the complete server on Windows, use [docs/docker.md](../docs/docker.md).
`setup_and_run.bat` is a browser/viewer launcher; it does not install the Docker
server or attach USB. `setup_windows.ps1` configures the Docker Desktop Wi-Fi
firewall rules. WSL USB requires usbipd attachment and a Docker daemon in the
same Ubuntu distribution, as described in the guide.

## Run without Docker

See [the complete native guide](../docs/native.md) for Ubuntu 22.04 with ROS 2
Humble, or Windows WSL2 Ubuntu with USB attachment. It covers dependencies,
agent/workspace builds, firmware, manual launch, services, backup, and updates.
On a Windows browser-only PC, open the existing server URL; no ROS or WSL is
needed there. The native installer starts services, so do not run a duplicate
manual launch alongside them.
