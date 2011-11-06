#!/sbin/sh

/sbin/busybox fbset -fb /dev/graphics/fb1 -n -g 18 18 18 18 32

case $1 in 
    480p)
        /sbin/busybox fbset -fb /dev/graphics/fb0 -n -g 720 480 720 960 16 -rgba 5/11,6/5,5/0,0/0
        echo $1 > /sys/class/display/mode
    ;;

    720p)
        /sbin/busybox fbset -fb /dev/graphics/fb0 -n -g 800 480 800 960 16 -rgba 5/11,6/5,5/0,0/0
        echo $1 > /sys/class/display/mode
        echo 240 120 800 480 240 120 18 18 > /sys/class/display/axis
    ;;

    1080p)
        /sbin/busybox fbset -fb /dev/graphics/fb0 -n -g 800 480 800 960 16 -rgba 5/11,6/5,5/0,0/0
        echo $1 > /sys/class/display/mode
        echo 160 60 800 480 160 60 18 18 > /sys/class/display/axis
    ;;

    panel)
        /sbin/busybox fbset -fb /dev/graphics/fb0 -n -g 800 480 800 960 16 -rgba 5/11,6/5,5/0,0/0
        echo $1 > /sys/class/display/mode
        echo 0 0 800 480 0 0 18 18 > /sys/class/display/axis
    ;;
    
    *)
        echo "Error: Un-supported display mode $1"
        echo "       Default to 720p"
        /system/bin/busybox fbset -fb /dev/graphics/fb0 -n -g 800 480 800 960 16 -rgba 5/11,6/5,5/0,0/0
        echo $1 > /sys/class/display/mode
        echo 240 120 800 480 240 120 18 18 > /sys/class/display/axis
esac
