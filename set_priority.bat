wmic process where name="javaw.exe" CALL setpriority "realtime"
wmic process where name="java.exe" CALL setpriority "realtime"
exit
