PORT1=/dev/cu.usbmodem101
PORT2=/dev/cu.usbmodem1101
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
	-pkill -f "pio device monitor --port /dev/.*usbmodem" || true

monitor1:
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT1) --baud 115200"'

monitor2:
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT2) --baud 115200"'

monitor-all: kill-monitors
	@ports=$$(ls /dev/tty.usbmodem* /dev/cu.usbmodem* 2>/dev/null || true); \
	if [ -z "$$ports" ]; then \
		echo "No usbmodem serial ports found (looked for /dev/*usbmodem*)."; \
		exit 1; \
	fi; \
	for p in $$ports; do \
		osascript -e "tell application \"Terminal\" to do script \"cd \\\"$(CURDIR)\\\" && pio device monitor --port $$p --baud 115200\""; \
	done

upload-monitor1: upload1
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT1) --baud 115200"'

upload-monitor2: upload2
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT2) --baud 115200"'

upload-monitor-all: kill-monitors upload-all
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT1) --baud 115200"'
	osascript -e 'tell application "Terminal" to do script "cd \"$(CURDIR)\" && pio device monitor --port $(PORT2) --baud 115200"'
