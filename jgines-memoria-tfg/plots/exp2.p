# don't make any output just yet
# plot the data file to get information on ranges
set terminal unknown
plot 'exp2.dat' using 1:2
xmin = GPVAL_DATA_X_MIN

set xlabel "Time (seconds)"
set xrange [0:36]

set ylabel "Quality"
set yrange [0.98:1.005]

set terminal png size 640,480 
set output 'exp2.png'

plot 'exp2.dat' using (($1-xmin)/1000):2 title "Post 1" with lines,\
     'exp2.dat' using (($1-xmin)/1000):3 title "Post 2" with lines,\
     'exp2.dat' using (($1-xmin)/1000):4 title "Post 3" with lines 