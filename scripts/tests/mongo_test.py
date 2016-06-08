import roslib
import rospy
from pymongo import MongoClient
if __name__ == '__main__':

    print("opening connection to bettyr")
    client = MongoClient('localhost',62345)
    print("gotcha")
    db = client['world_state']
    objects = db['Objects']
    res = db.Objects.find({})
    for k in res:
        print("object:")
        print(k)
        print("\n")
    print("done!")
