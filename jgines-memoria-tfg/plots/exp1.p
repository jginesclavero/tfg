# don't make any output just yet
# plot the data file to get information on ranges
set terminal unknown
plot 'exp1.dat' using 1:2 title "Object" with lines
xmin = GPVAL_DATA_X_MIN

set xlabel "Time (seconds)"
set xrange [0:36]

set ylabel "Quality"
set yrange [0.5:1.1]

set terminal png size 640,480 
set output 'exp1.png'

plot 'exp1.dat' using (($1-xmin)/1000):2 title "Object" with lines