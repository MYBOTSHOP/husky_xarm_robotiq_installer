import requests

r = requests.put('http://os-122212001984.local/api/v1/system/network/ipv4/override', headers={'Content-Type': 'application/json'}, json='192.168.132.1/24')
print(r)
print(r.content)
