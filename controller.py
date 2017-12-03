import bluetooth

nearby_devices = bluetooth.discover_devices()

num = 0
for i in nearby_devices:
	num += 1
	print num , ": " , bluetooth.lookup_name(i)

print "Make selection for Sparki Express #1..."
selection = input("> ") - 1
print bluetooth.lookup_name(nearby_devices[selection]) , " Selected"
bot1 = nearby_devices[selection]
