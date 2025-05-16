clear;
clc;






start_time = datetime("now");
now_time = datetime("now");

pause(1);

now_time = datetime("now");

app_time = seconds(now_time - start_time) + 60 * minutes(now_time - start_time) * 3600 * hours(now_time - start_time)


% app_time_sec = second(app_time) + 60 * minute(app_time) * 3600 * hour(app_time)




