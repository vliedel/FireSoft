#!/bin/sh
#
# Script to copy a module, so that you have the basic layout
#

template=templateModule

if [ "$1" = "" ]; then
	echo "Usage: ./createModule.sh <moduleName>"
	exit 1
fi

if [ -s $1 ]; then
	echo "${1} already exists!"
	exit 1
fi

basedir=${PWD}

tclass=($template)
tclass=${tclass[@]^}
tlower=${template,,}
tupper=${template^^}

echo $tlower $tclass $tupper

lower=${1,,}
upper=${1^^}
class=($1)
class=${class[@]^}

echo $lower $class $upper

# Copy files
rsync -r --exclude=.svn --exclude=build --exclude=aim/src/* --exclude=aim/inc/* ${template}/* $1

# Edit the files
#sed -re 's/match/replace/g' infile > outfile
sed -i -r "s/${template}/${1}/g" ${1}/aim/idl/${template}.idl
mv ${1}/aim/idl/${template}.idl ${1}/aim/idl/${1}.idl

sed -i -r "s/${template}/${1}/g" ${1}/aim/scripts/build.sh

sed -i -r "s/${template}/${1}/g" ${1}/CMakeLists.txt

sed -i -r "s/${template}/${1}/g" ${1}/inc/C${tclass}.h
sed -i -r "s/${tclass}/${class}/g" ${1}/inc/C${tclass}.h
sed -i -r "s/${tupper}/${upper}/g" ${1}/inc/C${tclass}.h
mv ${1}/inc/C${tclass}.h ${1}/inc/C${class}.h

sed -i -r "s/${template}/${1}/g" ${1}/src/C${tclass}.cpp
sed -i -r "s/${tclass}/${class}/g" ${1}/src/C${tclass}.cpp
mv ${1}/src/C${tclass}.cpp ${1}/src/C${class}.cpp

sed -i -r "s/${tclass}/${class}/g" ${1}/main/main.cpp
sed -i -r "s/${template}/${1}/g" ${1}/main/main.cpp

# SVN settings
exit 0

echo "Add \"${1}\" to svn?"
read go
if [ "$go" == "n" ]; then
	exit 0
fi

cd $basedir
svn add -N $1
cd ${basedir}/${1}
svn propset svn:ignore -F svn_ignore.txt .
svn add -N aim
cd ${basedir}/${1}/aim
svn propset svn:ignore -F svn_ignore.txt .
svn add *
cd ${basedir}/${1}
svn add *
svn stat
