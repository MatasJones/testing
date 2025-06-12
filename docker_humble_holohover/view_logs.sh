#!/bash/bin

PATH_TO_LOGS="/home/ubuntu/Documents/holohover_latency/testing/testing_logs/logger.csv"

### Change file location and name
PATH_TO_LOCAL_LOGS="/Users/matasjones/Desktop/PDS_II/holo_tests/week16/logger.csv"
file_name_first_part="log viewer"
###

CMD="scp $PATH_TO_LOGS $PATH_TO_LOCAL_LOGS"

echo "Fetching logs.."

source config.sh

# REMOTE_IPS[1] corresponds to the black holohover
if ping -c 1 ${SERVER_IP} &> /dev/null; then # ping -c 1 ip_address -> sends a ping to the ip address and &> /dev/null redirects any output to /dev/null effectively hiding it
    
    # Send command to devices
    
    scp ${USERS[1]}@${SERVER_IP}:${PATH_TO_LOGS} ${PATH_TO_LOCAL_LOGS}

else
    echo "Ping to $SERVER_IP failed..."

fi # Used to indicated the end of an if statement block

python3 /Users/matasjones/Documents/Coding_projects/testing/testing_logs/graph_logs.py $PATH_TO_LOCAL_LOGS "viewer"