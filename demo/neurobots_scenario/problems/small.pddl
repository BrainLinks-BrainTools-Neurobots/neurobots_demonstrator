(define (problem scenario_small)
 (:domain neurobots-small)

 (:objects
   kitchen living-room - room
   omnirob - robot
   shelf1 shelf2 - shelf
   ballon wwineglas cylinder beerstein - shape
   red white yellow blue green - color
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
   (= (at omnirob) shelf1)
   (arm-empty omnirob)
   (= (position c0) shelf1)
   (= (colored c0) red)
   (= (position g0) shelf2)
   (= (shaped g0) wwineglas)
   (= (position g1) shelf1)
   (= (shaped g1) beerstein)
 )

 (:goal (and)
)
)
