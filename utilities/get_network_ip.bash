interface=$1
ip_address=$(ip -o -4 addr show $interface | awk '{split($4, a, "/"); print a[1]}')
echo $ip_address
