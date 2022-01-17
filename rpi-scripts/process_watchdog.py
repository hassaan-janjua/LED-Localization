import subprocess
import os
import time
import datetime

localizer = subprocess.getoutput('pidof localizer')
doReboot = False
Message = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S - ')

if localizer is None or not localizer:
  doReboot = True
  Message += "The localizer process is not running."
else:
  cputime=int(subprocess.getoutput("cat /proc/uptime | cut -f1 -d \" \" | sed 's/\\.//'"))
  proctime=int(subprocess.getoutput("cat /proc/%d/stat | awk '{t = $14 + $15;print t}'" % int(localizer)))
  
  time.sleep(5)

  cputime2=int(subprocess.getoutput("cat /proc/uptime | cut -f1 -d \" \" | sed 's/\\.//'"))
  proctime2=int(subprocess.getoutput("cat /proc/%d/stat | awk '{t = $14 + $15;print t}'" % int(localizer)))
  
  
  cpu=(((proctime2-proctime)*100/(cputime2-cputime)))

  if cpu < 16.0:
    doReboot = True
    Message += "%f%%" % cpu
  else:
    Message += "%f%%" % cpu

print(Message)

if (doReboot):
  f = open("/home/pi/localization/process_watchdog_hb.txt", "a")
  f.seek(0, 2)
  f.write(Message)
  f.write("\n")
  f.close()
  os.system('reboot')
