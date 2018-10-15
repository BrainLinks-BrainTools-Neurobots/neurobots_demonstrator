#!/usr/bin/env python

import os
import rospy, rospkg, roslaunch
import random
import subprocess
import numpy
import sys

WORLDOBJS = 14
# (CUPS, GLASSES, BOTTLES, VASES, FLOWERS) = (5, 12, 12, 2, 4)

# always: 
# 5 rooms: pantry, kitchen, livin-groom, bathroom, garden
# 1 robot: omnirob
# 2 humans: me, friend
# 3 shelves: 2 in pantry 1 in bathroom
# 2 tables: couchtable diningtable
# 4 shapses: ballon wwineglas cylinder beerstein


if __name__ == "__main__":
    print (sys.argv)
    if len(sys.argv) >= 2:
        try:
            WORLDOBJS=int(sys.argv[1])
        except ValueError:
            print "1st parameter given to simgen was not an int, using %s world objects instead" % WORLDOBJS
    CGBVF = (0.15, 0.3, 0.2, 0.15, 0.2) # must add up to 1.0 (100%)
    assert sum(CGBVF) == 1.0, "object percentages do not add up to 100%"
    object_nums = numpy.diff(numpy.insert(numpy.rint(numpy.cumsum(CGBVF)* WORLDOBJS),0,0).astype(int))
    assert sum(object_nums) == WORLDOBJS, "generated objects do not match desired number"
    if len(sys.argv) >= 6:
        try:
            file_obj_nums = [int(sys.argv[i+1]) for i in range(5)]
            WORLDOBJS = sum(file_obj_nums)
            object_nums = file_obj_nums
        except ValueError:
            print "could not match parameters to cups, glasses, bottles, vases and flowers" % WORLDOBJS
    (CUPS, GLASSES, BOTTLES, VASES, FLOWERS) = object_nums

    rospy.init_node('scenario_gen_node')
    print("Creating a simulated scenario instance with %s world objects..." % WORLDOBJS)
    print("%s cups, %s glasses, %s bottles, %s vases and %s flowers" % (CUPS, GLASSES, BOTTLES, VASES, FLOWERS))
    rpkg = rospkg.RosPack()
    spath = rpkg.get_path('neurobots_scenario')
    sfile = os.path.join(spath, 'simgen.pddl') 

    file = open(sfile,"w")
    file.write("(define (problem scenario_gen_%s_%s_%s_%s_%s)\n (:domain neurobots-simgen)\n\n (:objects\n"%(CUPS,GLASSES,BOTTLES,VASES,FLOWERS))
    file.write("   kitchen living-room pantry bathroom garden - room\n")
    file.write("   omnirob - robot\n")
    file.write("   me friend - human\n")
    file.write("   flower-bed - flowerbed\n")
    file.write("   shelf1 shelf2 shelf3 - shelf\n")
    file.write("   dining-table couch-table - table\n")
    file.write("   ballon wwineglas cylinder beerstein - shape\n")
    file.write("   water lemonade apple-juice orange-juice red-wine white-wine beer - content\n")
    file.write("   red white yellow blue green - color\n")
    file.write("   left right - alignment\n")
    file.write("   c%s - cup\n"% " c".join(str(i) for i in range(CUPS)))
    file.write("   g%s - glass\n"% " g".join(str(i) for i in range(GLASSES)))
    file.write("   b%s - bottle\n"% " b".join(str(i) for i in range(BOTTLES)))
    file.write("   v%s - vase\n"% " v".join(str(i) for i in range(VASES)))        
    tulips = range(0,int(.3*FLOWERS))
    if tulips: 
        file.write("   f%s - tulip\n"% " f".join(str(i) for i in tulips))
    sunflowers = range(int(.3*FLOWERS),int(.6*FLOWERS))
    if sunflowers: 
        file.write("   f%s - sunflower\n"% " f".join(str(i) for i in sunflowers))
    roses = range(int(.6*FLOWERS),FLOWERS)
    file.write("   f%s - rose\n"% " f".join(str(i) for i in roses))
    file.write(" )\n\n (:init\n")
    rooms=["kitchen", "living-room", "pantry", "bathroom", "garden"]
    for r in rooms: 
        file.write("   (connected %s %s)\n"%(r,r))    
    connections = [("kitchen","pantry"), ("kitchen","living-room"), ("living-room","garden"), ("living-room", "bathroom")]
    for c in connections:        
        file.write("   (connected %s %s)\n"%c)
        file.write("   (connected %s %s)\n"%(c[1],c[0]))
    file.write("   (= (in shelf1) pantry)\n")
    file.write("   (= (aligned shelf1) left)\n")
    file.write("   (= (in shelf2) pantry)\n")
    file.write("   (= (aligned shelf2) right)\n")
    file.write("   (= (in shelf3) bathroom)\n")
    file.write("   (= (aligned shelf3) left)\n")
    file.write("   (= (in flower-bed) garden)\n")
    file.write("   (= (in dining-table) kitchen)\n")
    file.write("   (= (in couch-table) living-room)\n")
    file.write("   (= (in me) living-room)\n")
    file.write("   (= (in friend) living-room)\n")
    file.write("   (= (in omnirob) kitchen)\n")
    file.write("   (= (at omnirob) nowhere)\n")    
    file.write("   (mobile omnirob)\n")
    file.write("   (arm-empty omnirob)\n")
    bathroomflowers = dict()
    shapes = ["ballon", "wwineglas", "cylinder", "beerstein"]
    furnitures = ["dining-table", "couch-table", "shelf1", "shelf1", "shelf2", "shelf2", "shelf3"]
    opendrink = ["water", "empty", "empty", "empty", "empty", "empty"]
    closeddrink = ["water", "lemonade", "apple-juice", "orange-juice", "red-wine", "white-wine", "beer"]
    cupcolors = ["red", "white", "yellow", "blue", "green"]  
    flowercolors = ["red", "white", "yellow"]
    for i in range(CUPS):
        file.write("   (= (position c%s) %s)\n" % (i,random.choice(furnitures)))
        file.write("   (is-open c%s)\n" % i)         
        file.write("   (= (contains c%s) %s)\n" %  (i,random.choice(opendrink)))
        file.write("   (= (colored c%s) %s)\n" %  (i,random.choice(cupcolors)))
    for i in range(GLASSES):
        file.write("   (= (position g%s) %s)\n" % (i,random.choice(furnitures)))
        file.write("   (is-open g%s)\n" % i)         
        file.write("   (= (contains g%s) %s)\n" %  (i,random.choice(opendrink)))
        file.write("   (= (shaped g%s) %s)\n" %  (i,random.choice(shapes)))
    for i in range(BOTTLES):
        file.write("   (= (position b%s) %s)\n" % (i,random.choice(furnitures)))        
        file.write("   (= (contains b%s) %s)\n" %  (i,random.choice(closeddrink)))
    for i in range(VASES):
        furn = random.choice(furnitures)
        file.write("   (= (position v%s) %s)\n" % (i, furn))
        vasecontent= "empty"
        if furn == "shelf3":
            bathroomflower=random.randint(0,FLOWERS-1)
            if (not bathroomflower in bathroomflowers):
                bathroomflowers[bathroomflower]="v%s"%i;
                vasecontent="f%s"%bathroomflower
        file.write("   (= (contains v%s) %s)\n" %  (i,vasecontent))
        file.write("   (= (colored v%s) %s)\n" %  (i,random.choice(cupcolors)))                        
    for i in range(FLOWERS):
        flcolor = random.choice(flowercolors)
        if i in sunflowers:
            flcolor = "yellow"
        file.write("   (= (colored f%s) %s)\n" %  (i,flcolor))
        flowerpos = bathroomflowers.get(i,"flower-bed")
        file.write("   (= (position f%s) %s)\n" %  (i,flowerpos))            
    file.write(" )\n\n (:goal (and)\n)\n)")
    file.close()    
    print("Creation of instance done, file written to %s"%file)
    print("Starting database now...")
    
#     package = 'neurobots_database'
#     executable = 'neurobots_database_node'
#     arguments = 'simgen'
#     node = roslaunch.core.Node(package, executable, arguments)
#     launch = roslaunch.scriptapi.ROSLaunch()
#     launch.start()
# 
#     process = launch.launch(node)
#     print process.is_alive()
#     process.stop()

 
#     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#     roslaunch.configure_logging(uuid)
#     lpath = rpkg.get_path('neurobots_launch')
#     lfile = os.path.join(lpath, 'launch/database/database_simgen.launch') 
#     launch = roslaunch.parent.ROSLaunchParent(uuid, [lfile])
#      
#     launch.start()
#  
#    launch.shutdown()
    subprocess.call(["roslaunch","neurobots_launch", "database_simgen.launch"])
