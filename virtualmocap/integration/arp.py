import subprocess
import re


def get_arp_table():
    arp_output = subprocess.check_output(["arp", "-a"], text=True)

    # Regex for parsing IP and MAC addresses
    ip_mac_pattern = re.compile(
        r"\(([\d.]+)\)\s+at\s+([0-9a-f:]{17}|[0-9a-f-]{17})", re.IGNORECASE
    )

    arp_entries = ip_mac_pattern.findall(arp_output)
    return [(ip, mac.upper().replace("-", ":")) for ip, mac in arp_entries]


def get_mac_mapping(mac_list):
    mac_list = [mac.upper() for mac in mac_list]
    arp_entries = get_arp_table()

    mac_to_ip, ip_to_mac = {}, {}
    for mac in mac_list:
        for ip, arp_mac in arp_entries:
            if mac == arp_mac:
                mac_to_ip[mac] = ip
                ip_to_mac[ip] = mac
                break  # Found the IP, no need to check further

    return mac_to_ip, ip_to_mac
