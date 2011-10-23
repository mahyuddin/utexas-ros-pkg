if [ $# -ne 1 ]
then
  echo "Usage: $0 <your-google-account-name>"
  exit
fi
cd ~/svn
svn co https://utexas-ros-pkg.googlecode.com/svn/branches/experimental/config/icewm icewm-config --username $1
rm -rf ~/.icewm
ln -s `pwd`/icewm-config ~/.icewm
