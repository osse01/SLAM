set multiplot layout 1,2
set title "X trajectory, particle filter"
set xlabel "x_1"
set ylabel "x_2"
set key auto
plot "particle_filter.dat" with linespoints lw 2

set title "X trajectory ekf"
set xlabel "x_1"
set ylabel "x_2"
set key auto
plot "ekf.dat" with linespoints lw 2

