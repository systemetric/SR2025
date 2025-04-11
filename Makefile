TARGET_DEVICE = ""

all: copy-files

copy-files:
	cp ./* $(TARGET_DEVICE)
    sudo umount $(TARGET_DEVICE)

clean:
	rm -rf $(TARGET_DEVICE)/*