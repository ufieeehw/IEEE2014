To use these files, you must create a symlink to them in /etc/udev/rules.d/

To reload the rules, run 
	sudo udevadm control --reload-rules

To trigger the name mappings, run
	sudo udevadm trigger

