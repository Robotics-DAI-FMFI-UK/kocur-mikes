#! /bin/sh
### BEGIN INIT INFO
# Provides:          mikes
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: mikes deamon
# Description:       all background services for tomcat Mikes
### END INIT INFO

# this file location: /etc/init.d/mikes

case "$1" in
  start)
    echo "Starting mikes "
    sudo -u pi nohup /usr/local/bin/mikes autostart &
    ;;
  stop)
    echo "Stopping mikes"
    killall -s 15 mikes
    ;;
  *)
    echo "Usage: /etc/init.d/mikes {start|stop}"
    exit 1
    ;;
esac

exit 0

