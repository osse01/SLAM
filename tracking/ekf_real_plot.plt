set title "Figure-Eight Trajectory Comparison"
set xlabel "x_1 (meters)"
set ylabel "x_2 (meters)"
set key auto
set grid
set terminal pngcairo
set output 'ekf_plot.png'
plot "../data/mic_locations.txt" using 1:2 with points pt 9 ps 3 lc rgb "red" title "Microphones", \
     "../data/position_formatted.dat" with linespoints lw 0.1 pt 7 ps 0.5 lc rgb "red" title "True Trajectory", \
     "ekf_real.dat" with linespoints lw 2 pt 7 ps 0.5 title "EKF Trajectory"
unset output