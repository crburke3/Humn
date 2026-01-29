PORT1=/dev/tty.usbmodem21101
PORT2=/dev/tty.usbmodem21401
ENV=wireless_tracker

.PHONY: upload1 upload2 upload-all monitor1 monitor2 monitor-all upload-monitor1 upload-monitor2 upload-monitor-all kill-monitors

upload1:
	pio run -t upload -e $(ENV) --upload-port $(PORT1)

upload2:
	pio run -t upload -e $(ENV) --upload-port $(PORT2)

upload-all:
	pio run -t upload -e $(ENV) --upload-port $(PORT1)
	pio run -t upload -e $(ENV) --upload-port $(PORT2)

kill-monitors:
	-pkill -f "pio device monitor --port $(PORT1)" || true
	-pkill -f "pio device monitor --port $(PORT2)" || true

monitor1:
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT1) --baud 115200"'

monitor2:
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT2) --baud 115200"'

monitor-all: kill-monitors
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT1) --baud 115200"'
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT2) --baud 115200"'

upload-monitor1: upload1
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT1) --baud 115200"'

upload-monitor2: upload2
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT2) --baud 115200"'

upload-monitor-all: kill-monitors upload-all
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT1) --baud 115200"'
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT2) --baud 115200"'
