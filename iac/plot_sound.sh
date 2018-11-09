#!/bin/bash

grep Rep_sound log > sound.log
echo 'plot [][-1:101] "sound.log" u :2 w l, "sound.log" u :3 w l, "sound.log" u :4 w l' > sound.gnuplot
echo 'pause -1' >> sound.gnuplot
gnuplot sound.gnuplot
