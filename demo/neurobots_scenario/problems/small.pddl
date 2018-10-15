(define (problem scenario_small)
 (:domain neurobots-small)

 (:objects
   kitchen living-room - room
   omnirob - robot
   shelf1 shelf2 - shelf
   blue green - color
   c0 - cup
   g0 g1 - glass
 )

 (:init
   (connected kitchen kitchen)
   (connected living-room living-room)
   (connected kitchen living-room)
   (connected living-room kitchen)
   (= (in shelf1) kitchen)
   (= (in shelf2) living-room)
   (= (in omnirob) kitchen)
   (= (at omnirob) nowhere)
   (arm-empty omnirob)
   (= (position c0) shelf1)
   (= (colored c0) blue)
   (= (position g0) shelf2)
   (= (position g1) shelf1)
 )

 (:goal (and)
)
)
