# Real-time LIDAR plotting
set terminal x11 size 800,800
set title "Real-time LIDAR Scan"

set polar
set angles degrees
set grid polar
set size square
set rrange [0:4000]
set trange [0:360]

while (1) {
    plot "lidar_data.txt" using 1:2 with points pt 7 ps 1 lc rgb "red" title "LIDAR Points"
    pause 0.2
}