#!/bin/bash
set -e

# Color Variables
NOCOLOR='\033[0m'
RED='\033[1;31m'
GREEN='\033[1;32m'
BLUE='\033[1;34m'
YELLOW='\033[1;33m'
WHITE='\033[1;37m'

# Variables
LAB_NAME="lab6"

PWD=$(pwd)
HOME_DIR=$HOME"/"
SOURCE_DIR=$PWD"/"
TARGET_DIR=$HOME_DIR"catkin_ws/src/"
CATKIN_DIR=$HOME_DIR"catkin_ws/"
WORK_DIR=$TARGET_DIR$LAB_NAME"/"

LOG_FILE=$SOURCE_DIR"output_"$LAB_NAME".log"

# Project name
ZIP_FILE=$LAB_NAME".zip"

function print_if_file_exists
{
  if [ -f "$1" ]; then
      echo "   $1"
      echo "   $1" >> "$LOG_FILE"
  fi
}


################## CHECK FILES ###################
echo -e "${BLUE} > Log output written to: $LOG_FILE ${NOCOLOR}"
echo -e "${BLUE} > Lab Work Directory: $TARGET_DIR$LAB_NAME/scripts/ ${NOCOLOR}"

# Log output (INIT)
echo " > Lab Work Directory: $TARGET_DIR" > "$LOG_FILE"

echo -e "${YELLOW}Validating...${NOCOLOR}"
# Test if in the right directory
FOUND_ZIP=1
if [ ! -f "$LAB_NAME.zip" ]; then
    echo -e "   ${RED}Cannot find "$LAB_NAME".zip in the current directory.${NOCOLOR}"
    echo "Cannot find "$LAB_NAME".zip in the current directory." >> "$LOG_FILE"
    FOUND_ZIP=0
fi

FOUND_ALISES=1
if [ ! -f "bash_aliases" ]; then
    echo -e "   ${RED}Cannot find bash_aliases in the current directory.${NOCOLOR}"
    echo "Cannot find bash_aliases in the current directory." >> "$LOG_FILE"
    FOUND_ALISES=0
fi
if [[ "$FOUND_ZIP" -eq "0" || "$FOUND_ALISES" -eq "0" ]]; then
   echo -e "${RED}Please check if you are running the setup script from within the "$LAB_NAME"_base_code directory.${NOCOLOR}"
   echo "Exiting..."
   exit
fi

# Check if files exist
if [ -d "$WORK_DIR" ]; then
    echo ""
    echo "The $LAB_NAME directory already exists."
    echo "All the $LAB_NAME files, including the following files will be overwritten if you continue: "
    print_if_file_exists $WORK_DIR"scripts/robot_interface.py"
    print_if_file_exists $WORK_DIR"scripts/$LAB_NAME.ipynb"
    echo ""
    while true; do
      read -p "Are you sure you want to overwrite the files?(y/n)" yn
      case $yn in
          [Yy]* ) break;;
          [Nn]* ) echo "Exiting..."; exit;;
          * ) echo "Please answer yes or no.";;
      esac
    done
fi
###################################################


################## EXTRACT FILES ##################
echo -e "${YELLOW}Step 1/3: Extracting Files to: $TARGET_DIR ${NOCOLOR}"

# Log output
echo "------- STEP: 1 -------" >> "$LOG_FILE"

# Unzip package into catkin workspace
unzip -o $ZIP_FILE -d $TARGET_DIR >> "$LOG_FILE"
###################################################


################## SETUP ALIASES ##################
echo -e "${YELLOW}Step 2/3: Setting up commands${NOCOLOR}"

# Remove aliases file
rm -f $HOME_DIR/.bash_aliases

# Create aliases file
cp "bash_aliases" $HOME/.bash_aliases

# Add line to .bashrc to source aliases file
grep -qxF "source ~/.bash_aliases" $HOME/.bashrc || echo -e "\nsource ~/.bash_aliases" >> $HOME/.bashrc

# Log output
echo "------- STEP: 2 -------" >> "$LOG_FILE"
tail -5 $HOME/.bashrc >> "$LOG_FILE"
echo "--- ls:" >> "$LOG_FILE"
ls -ltr $HOME/.bash_aliases &>> "$LOG_FILE"
echo "--- cat:" >> "$LOG_FILE"
cat $HOME/.bash_aliases &>> "$LOG_FILE"
echo "" >> "$LOG_FILE"
###################################################

################## CATKIN_MAKE ##################
echo -e "${YELLOW}Step 3/3: Compiling Project${NOCOLOR}"
mkdir -p "${TARGET_DIR}"

cd "${CATKIN_DIR}"

set +e
# Log output
echo "------- STEP: 3 -------" >> "$LOG_FILE"
catkin_make &>> "$LOG_FILE"

if [ $? -eq 0 ]
then
  echo ""
  echo -e "${GREEN}Successfully compiled lab.${NOCOLOR}"
  echo -e "${YELLOW}NOTE: Make sure you close all terminals after this message.${NOCOLOR}"
  
  source $HOME_DIR"/.bashrc"
else
  echo -e "${RED}FAILURE: Could not setup lab base code.${NOCOLOR}"
  echo "Log output written to: "$LOG_FILE
fi
##################################################