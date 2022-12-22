import requests

r = requests.put('http://os-122140000011.local/api/v1/system/network/ipv4/override', headers={'Content-Type': 'application/json'}, json='192.168.131.2/24')
print(r)
print(r.content)
