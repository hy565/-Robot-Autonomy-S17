#!/usr/bin/env python2

import numpy as np

def print_list(l):
    print (l)

def sort_manual(shops):

    shops_sorted = []

    # print shops.keys()
    first = shops.keys()[0]
    # print shops.get(first)
    shops_sorted.append([first, shops.get(first)])
    # TODO: Here implement manual sorting using loops
	# for shop in shops.keys():
    	# print (shops[shop])
    for shop in shops.keys()[1:]:
    	# print shop
        i = 0

        while  i < len(shops_sorted):
            if (shops.get(shop) > shops_sorted[i][1]):
                shops_sorted.insert(i, [shop, shops.get(shop)])
                break        
            else:
                i+=1

        if (i == len(shops_sorted)):
            shops_sorted.insert(i+1, [shop, shops.get(shop)])
        # print shops_sorted

    print ('Manual sorting result: ')
    print_list(shops_sorted)

def sort_python(shops):
    #TODO: Here implement sorting using pythons build in sorting functions
    shops_sorted = []
    shops_lists =  [list(i) for i in shops.items()]
    shops_sorted = sorted(shops_lists, key=lambda shop: shop[1], reverse=True)
    
    print 'Python sorting result: '
    print_list(shops_sorted)

def sort_numpy(shops):
    
    shops_sorted = []
    
    # x = np.array(shops.values())
    # print np.argsort(x)
    shops_sorted = [[shops.keys()[i], shops.values()[i]] for i in np.argsort(np.array(shops.values()))][::-1]

    # d = [('name', 'S20'), ('val', float)]
    # shops_lists = np.array(shops.items(), dtype=d)
    # print shops_lists
    # shops_sorted = list(np.sort(shops_lists, order = 'val'))
    
    # TODO: Here implement sorting using numpys build-in sorting function
    print 'Numpy sorting result: '
    print_list(shops_sorted)

def main():

    shops = {}
    shops['21st Street'] = 0.9
    shops['Voluto'] = 0.6
    shops['Coffee Tree'] = 0.45
    shops['Tazza D\' Oro'] = 0.75
    shops['Espresso a Mano'] = 0.95
    shops['Crazy Mocha'] = 0.35
    shops['Commonplace'] = 0.5

    sort_manual(shops)
    sort_python(shops)
    sort_numpy(shops)
    

if __name__ == "__main__":
    main()
