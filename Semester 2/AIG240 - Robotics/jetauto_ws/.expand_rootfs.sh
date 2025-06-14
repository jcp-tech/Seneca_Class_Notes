#!/bin/bash
sleep 5

do_expand_fs() {
    PARTITION=$1
    echo "Expanding $PARTITION..."
    PARTITION_NUM=${PARTITION##*blk*p}
    DISK=${PARTITION%%p*}

    # Get the starting offset of the root partition
    PART_START=$(sudo parted ${DISK} -ms unit s p | grep "^${PARTITION_NUM}:" | cut -f 2 -d: | sed 's/[^0-9]//g')
    [ "$PART_START" ] || return 1
    echo $PART_START
    echo -e "d\n$PARTITION_NUM\nn\n$PARTITION_NUM\n$PART_START\n\nN\n\nw\n\n"|fdisk $DISK 2>&1
    sync
    sleep 0.5
    resize2fs $PARTITION
}

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root!" 1>&2
    sudo $0 $@
fi

ROOTFS_STORAGE_TYPE="SDCARD"
BOARD_COMPATIBLE=$(tr -d '\0' < /proc/device-tree/compatible)
if [[ $BOARD_COMPATIBLE =~ "p3449-0000-b00+p3448-0002-b00" ]];then
    ROOTFS_STORAGE_TYPE="EMMC"
fi

if [[ $ROOTFS_STORAGE_TYPE == "EMMC" ]];then
    ROOTFS_STORAGE="/dev/mmcblk1"
    ROOTFS_PARTITION="/dev/mmcblk1p1"
    for _ in $(seq 20);do
        echo -e "w\ny\ny\n\n\n" |gdisk $ROOTFS_STORAGE
	sync
        do_expand_fs $ROOTFS_PARTITION
	sync
	ROOTFS_PARTITION_SIZE=$(df -BG|grep "$ROOTFS_PARTITION" |awk '{print $2}' |tr -d 'G')
	if [[ $ROOTFS_PARTITION_SIZE -gt 29 ]];then
	    sync
	    break
	else
	    sleep 2
    	fi	    
    done
fi

if [[ $ROOTFS_STORAGE_TYPE == "SDCARD" ]];then
    ROOTFS_STORAGE="/dev/mmcblk0"
    ROOTFS_PARTITION="/dev/mmcblk0p1"
    for _ in $(seq 20);do
        echo -e "w\ny\ny\n\n\n" |gdisk $ROOTFS_STORAGE
	sync
        do_expand_fs $ROOTFS_PARTITION
	sync
	ROOTFS_PARTITION_SIZE=$(df -BG|grep "$ROOTFS_PARTITION" |awk '{print $2}' |tr -d 'G')
	if [[ $ROOTFS_PARTITION_SIZE -gt 29 ]];then
	    sync
	    break
	else
	    sleep 2
    	fi	    
    done
fi

if [[ !($@ =~ "no_reboot") ]];then
    sudo systemctl disable expand_rootfs.service
    reboot -d -f
fi
