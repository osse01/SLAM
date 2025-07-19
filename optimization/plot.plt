#set multiplot layout 1,2
#set title "X trajectory"
#set xlabel "x_1"
#set ylabel "x_2"
#set key auto
#plot "xdata.dat" with linespoints lw 2

set title "Residual norm"
set xlabel "iteration"
set ylabel "||r||_2^2"
set logscale y
set format y "10^{%L}"  # Scientific notation for Y-axis
set key auto
plot "fdata.dat" with linespoints lw 2
