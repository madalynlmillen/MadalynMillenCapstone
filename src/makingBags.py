import pyrosbag as rosbag

def makeABag(pointsList):
    bag = rosbag.Bag('path.bag', 'w')

    try:
        bag.write('points', pointsList)
    finally:
        bag.close()