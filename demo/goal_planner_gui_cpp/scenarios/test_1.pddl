(define (problem scenario_gen_2_4_3_2_3)
 (:domain neurobots-simgen)

 (:objects
   kitchen living-room pantry bathroom garden - room
   omnirob - robot
   me friend - human
   flower-bed - flowerbed
   shelf1 shelf2 shelf3 - shelf
   dining-table couch-table - table
   ballon wwineglas cylinder beerstein - shape
   water lemonade apple-juice orange-juice red-wine white-wine beer - content
   red white yellow blue green - color
   left right - alignment
   c0 c1 - cup
   g0 g1 g2 g3 - glass
   b0 b1 b2 - bottle
   v0 v1 - vase
   f0 - sunflower
   f1 f2 - rose
 )

 (:init
   (connected kitchen kitchen)
   (connected living-room living-room)
   (connected pantry pantry)
   (connected bathroom bathroom)
   (connected garden garden)
   (connected kitchen pantry)
   (connected pantry kitchen)
   (connected kitchen living-room)
   (connected living-room kitchen)
   (connected living-room garden)
   (connected garden living-room)
   (connected living-room bathroom)
   (connected bathroom living-room)
   (= (in shelf1) pantry)
   (= (aligned shelf1) left)
   (= (in shelf2) pantry)
   (= (aligned shelf2) right)
   (= (in shelf3) bathroom)
   (= (aligned shelf3) left)
   (= (in flower-bed) garden)
   (= (in dining-table) kitchen)
   (= (in couch-table) living-room)
   (= (in me) living-room)
   (= (in friend) living-room)
   (= (in omnirob) kitchen)
   (= (at omnirob) nowhere)
   (mobile omnirob)
   (arm-empty omnirob)
   (= (position c0) dining-table)
   (is-open c0)
   (= (contains c0) empty)
   (= (colored c0) red)
   (= (position c1) shelf2)
   (is-open c1)
   (= (contains c1) empty)
   (= (colored c1) blue)
   (= (position g0) dining-table)
   (is-open g0)
   (= (contains g0) empty)
   (= (shaped g0) cylinder)
   (= (position g1) shelf2)
   (is-open g1)
   (= (contains g1) empty)
   (= (shaped g1) ballon)
   (= (position g2) couch-table)
   (is-open g2)
   (= (contains g2) empty)
   (= (shaped g2) beerstein)
   (= (position g3) shelf1)
   (is-open g3)
   (= (contains g3) empty)
   (= (shaped g3) ballon)
   (= (position b0) shelf1)
   (= (contains b0) water)
   (= (position b1) shelf3)
   (= (contains b1) beer)
   (= (position b2) shelf3)
   (= (contains b2) water)
   (= (position v0) shelf2)
   (= (contains v0) empty)
   (= (colored v0) yellow)
   (= (position v1) shelf3)
   (= (contains v1) f1)
   (= (colored v1) green)
   (= (colored f0) yellow)
   (= (position f0) flower-bed)
   (= (colored f1) red)
   (= (position f1) v1)
   (= (colored f2) yellow)
   (= (position f2) flower-bed)
 )

 (:goal (and)
)
)