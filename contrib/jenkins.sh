#!/bin/sh -ex
# Build the kernel module against the linux tree in KSRC

DEFCONFIG="${DEFCONFIG:-x86_64_defconfig}"
TOPDIR="$(realpath "$(dirname "$(realpath "$0")")/..")"
KSRC="${KSRC:-$TOPDIR/../linux}"
JOBS="${JOBS:-9}"

if ! [ -d "$KSRC" ]; then
	set +x
	echo "ERROR: KSRC does not exist: $KSRC"
	echo "Let the KSRC env var point at a linux source tree and try again."
	exit 1
fi

cd "$KSRC"

if ! [ -e ".config" ]; then
	make "$DEFCONFIG"
fi

if [ "arch/x86/configs/$DEFCONFIG" -nt ".config" ]; then
	set +x
	echo "ERROR: .config inside kernel source tree is older than $DEFCONFIG"
	echo "Move/delete/touch .config and try again."
	exit 1
fi

git log -1 --pretty="%t - %s"

make -j "$JOBS" modules_prepare

# Wno-error=address: OS#6098
make \
	-j "$JOBS" \
	-C "$TOPDIR" \
	KSRC="$KSRC" \
	KBUILD_MODPOST_WARN=1 \
	KCFLAGS="-Wno-error=address"
