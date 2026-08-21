"""
PlatformIO custom upload script: flash the fire controller over HTTP.

Transport is HTTP, not espota, and deliberately so: macOS Sequoia blocks
espota's UDP, so an espota-only path would work on the device and fail at the
toolchain.  The same problem was already hit and solved in WLED_dev
(WLED/platformio.ini env:ampworks + tools/upload_wled.py); this is that
precedent applied here.

curl is used rather than Python's urllib for the same reason WLED does: the
Sequoia local-network restriction blocks Python but not system binaries.

The device refuses the upload unless every ignition switch is inactive, the
core-1 status snapshot is fresh, and the last switch read succeeded -- see
fc_ota_guard.h.  A refusal is an HTTP 409 with a JSON reason, which this script
surfaces rather than swallowing, so a blocked upload never looks like a hang.

Target IP comes from upload_port in platformio.ini, overridable at upload time:
    FC_OTA_IP=192.168.1.99 FC_OTA_PASS=<password> pio run -e touchcontroller_esp32_ota -t upload

FC_OTA_PASS must match the password the image was built with (-DFC_OTA_PASS).
"""
import os
import subprocess
import sys

Import("env")  # noqa: F821  (injected by PlatformIO)


def upload_ota(source, target, env):
    firmware = str(source[0])
    ip = os.environ.get("FC_OTA_IP", env.GetProjectOption("upload_port", ""))
    if not ip:
        print("error: no target IP -- set FC_OTA_IP or upload_port")
        sys.exit(1)

    password = os.environ.get("FC_OTA_PASS", "")
    if not password:
        print("error: FC_OTA_PASS is not set.  The device requires HTTP Basic "
              "auth (user 'ota') on /update; an unauthenticated upload is "
              "refused with 401.")
        sys.exit(1)

    url = "http://%s/update" % ip
    print("\nUploading %s to %s ..." % (firmware, url))

    cmd = [
        "curl", "-X", "POST", url,
        "--user", "ota:%s" % password,
        "-F", "update=@%s" % firmware,
        "--progress-bar",
        "--max-time", "120",
        "-w", "\nHTTP %{http_code}\n",
        # The refusal body carries the reason (which switch, stale snapshot,
        # failed switch read); print it instead of discarding it.
        "-o", "-",
    ]

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("curl failed with exit code %d" % result.returncode)
        sys.exit(1)
    print("Upload posted -- check the HTTP status above.  200 means the device "
          "accepted the image and is rebooting; 401 means bad credentials; "
          "409 means a guard refused it and the body says which.")


env.Replace(UPLOADCMD=upload_ota)  # noqa: F821
