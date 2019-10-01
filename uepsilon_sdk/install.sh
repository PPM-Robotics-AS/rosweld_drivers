#!/bin/bash

LOCAL_LIB_DIR='/usr/local/lib/'
INCLUDE_DIR='/usr/local/include/'
OPT_DIR='/opt/scanCONTROL/'

SDK='.'
BIN_DIR='/include/'
LIB_DIR='/lib/x86_64/'
MESCAN='/libmescan/'
LLT='/libllt/'
LLT_H='llt.h'
LLT_SO='libllt.so.0.2.0'
LLT_H2='LLTDataTypes.h'
MESCAN_H='mescan.h'
MESCAN_SO='libmescan.so.0.2.0'
DIR_ERROR="You have to run ./install.sh \"/path/to/SDK(Linux)/\""

if ! [ -d "$SDK$BIN_DIR$MESCAN" ]
then
    echo "Could not find $MESCAN folder in $SDK$BIN_DIR."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -d "$SDK$BIN_DIR$LLT" ]
then
    echo "Could not find $LLT folder in $SDK$BIN_DIR."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$SDK$BIN_DIR$MESCAN$MESCAN_H" ]
then
    echo "Could not find $MESCAN_H in $SDK$BIN_DIR$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$SDK$LIB_DIR$MESCAN_SO" ]
then
    echo "Could not find $MESCAN_SO in $SDK$LIB_DIR."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$SDK$BIN_DIR$LLT$LLT_H" ]
then
    echo "Could not find $LLT_H in $SDK$BIN_DIR$LLT."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$SDK$BIN_DIR$MESCAN$LLT_H2" ]
then
    echo "Could not find $LLT_H2 in $SDK$BIN_DIR$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$SDK$LIB_DIR$LLT_SO" ]
then
    echo "Could not find $LLT_SO in $SDK$LIB_DIR."
    echo $DIR_ERROR
    exit 1
fi

echo "Creating directories need sudo rights."
sudo mkdir -p -- "$INCLUDE_DIR$MESCAN"
sudo mkdir -p -- "$INCLUDE_DIR$LLT"
sudo mkdir -p -- "$OPT_DIR"

echo "Copying libs to $LOCAL_LIB_DIR, need sudo rights."
sudo cp -- $SDK$LIB_DIR$MESCAN_SO "$LOCAL_LIB_DIR"
sudo cp -- $SDK$LIB_DIR$LLT_SO "$LOCAL_LIB_DIR"
sudo chmod 755 "$LOCAL_LIB_DIR$LLT_SO"
sudo chmod 755 "$LOCAL_LIB_DIR$MESCAN_SO"

echo "running ldconfig to create symlinks and cache."
sudo ldconfig
sudo ln -s -- "$LOCAL_LIB_DIR$LLT_SO" "$LOCAL_LIB_DIR"libllt.so
sudo ln -s -- "$LOCAL_LIB_DIR$MESCAN_SO" "$LOCAL_LIB_DIR"libmescan.so

echo "Copying headers to $INCLUDE_DIR, need sudo rights."
sudo cp -- $SDK$BIN_DIR$MESCAN* "$INCLUDE_DIR$MESCAN"
sudo cp -- $SDK$BIN_DIR$LLT* "$INCLUDE_DIR$LLT"
sudo cp -- $SDK$BIN_DIR$MESCAN$LLT_H2 "$INCLUDE_DIR$LLT"
sudo chmod -R u+rwX,g+rX,o+rX "$INCLUDE_DIR$MESCAN"
sudo chmod -R u+rwX,g+rX,o+rX "$INCLUDE_DIR$LLT"

echo "Installing prerequisites for aravis-0.5, need sudo rights."
sudo apt-get install -y autoconf intltool automake gtk-doc-tools

echo "Downloading and extracting aravis-0.5 to your home."
cd ~
wget http://ftp.acc.umu.se/pub/GNOME/sources/aravis/0.5/aravis-0.5.10.tar.xz -O aravis-0.5.10.tar.xz
tar xfJ aravis-0.5.10.tar.xz
rm aravis-0.5.10.tar.xz

echo "Configuring, Compiling and Installing aravis-0.5."
cd aravis-0.5.10
./configure
make
sudo make install
