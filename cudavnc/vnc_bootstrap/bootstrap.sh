#!/bin/bash
supervisord -c /etc/supervisor/conf.d/supervisord.conf
/bin/bash
kill -SIGTERM `cat /tmp/supervisord.pid`