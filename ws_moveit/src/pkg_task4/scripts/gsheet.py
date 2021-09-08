#!/usr/bin/env python
import requests
import rospy
import json
from datetime import datetime , timedelta


class GSheets:

    def __init__(self):
        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

    def update_google_sheets(self,*args):

        #res = '{"city": "Amritsar", "order_time": "2021-01-30 15:58:42", "order_id": "1003", "lon": "74.8723 E", "qty": "1", "item": "Medicine", "lat": "31.6340 N"}'  
        #res1 = json.loads(res)
        #res2 = list(res1.values())
        #print(res2[0])
        
        

       
        


        # defining our sheet name in the 'id' variable and the the column where we want to update the value
        parameters = {"id":"OrdersShipped","Team Id":"VB_1299","Unique Id":"MnOpqRsT","Order Id":args[0][0],"City": args[0][3],"Item":args[0][1],"Priority": args[0][6],"Shipped Quantity":args[0][2],"Latitude": args[0][4] ,"Longitude": args[0][5] ,"Cost":args[0][7],"Shipped Status and time": args[0][8],"Estimated Time of Delivery":args[0][9] ,"Shipped Status": args[0][10]} 

        URL = "https://script.google.com/macros/s/AKfycbyuc519rp6UbeLF7lE8wyPT32MlU6zkllLapdqMhzBEohqH33u_RLA/exec"

        response = requests.get(URL, params=parameters)

        print(response.content)

def main():

    gs= GSheets()

    or1=["3001","Clothes","1","Mumbai","19.0760 N","72.8777 E","LP","150"]
    or1.append(datetime.now())
    add = datetime.now()+timedelta(5)
    or1.append(add)
    or1.append("YES")
    gs.update_google_sheets(or1)


if __name__ == '__main__':
    main()


