(define (problem scenario_gen_5_12_12_2_4)
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
   c0 c1 c2 c3 c4 - cup
   g0 g1 g2 g3 g4 g5 g6 g7 g8 g9 g10 g11 - glass
   b0 b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 b11 - bottle
   v0 v1 - vase
   f0 - tulip
   f1 - sunflower
   f2 f3 - rose
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
   (= (position c0) shelf2)
   (is-open c0)
   (= (contains c0) empty)
   (= (colored c0) white)
   (= (position c1) dining-table)
   (is-open c1)
   (= (contains c1) empty)
   (= (colored c1) blue)
   (= (position c2) shelf2)
   (is-open c2)
   (= (contains c2) empty)
   (= (colored c2) yellow)
   (= (position c3) shelf2)
   (is-open c3)
   (= (contains c3) water)
   (= (colored c3) green)
   (= (position c4) shelf2)
   (is-open c4)
   (= (contains c4) empty)
   (= (colored c4) white)
   (= (position g0) shelf1)
   (is-open g0)
   (= (contains g0) empty)
   (= (shaped g0) wwineglas)
   (= (position g1) couch-table)
   (is-open g1)
   (= (contains g1) empty)
   (= (shaped g1) wwineglas)
   (= (position g2) shelf1)
   (is-open g2)
   (= (contains g2) empty)
   (= (shaped g2) ballon)
   (= (position g3) shelf1)
   (is-open g3)
   (= (contains g3) empty)
   (= (shaped g3) cylinder)
   (= (position g4) dining-table)
   (is-open g4)
   (= (contains g4) empty)
   (= (shaped g4) beerstein)
   (= (position g5) couch-table)
   (is-open g5)
   (= (contains g5) water)
   (= (shaped g5) beerstein)
   (= (position g6) couch-table)
   (is-open g6)
   (= (contains g6) empty)
   (= (shaped g6) ballon)
   (= (position g7) shelf2)
   (is-open g7)
   (= (contains g7) empty)
   (= (shaped g7) beerstein)
   (= (position g8) shelf3)
   (is-open g8)
   (= (contains g8) empty)
   (= (shaped g8) cylinder)
   (= (position g9) shelf1)
   (is-open g9)
   (= (contains g9) empty)
   (= (shaped g9) cylinder)
   (= (position g10) couch-table)
   (is-open g10)
   (= (contains g10) empty)
   (= (shaped g10) beerstein)
   (= (position g11) shelf2)
   (is-open g11)
   (= (contains g11) empty)
   (= (shaped g11) beerstein)
   (= (position b0) couch-table)
   (= (contains b0) beer)
   (= (position b1) couch-table)
   (= (contains b1) water)
   (= (position b2) shelf3)
   (= (contains b2) red-wine)
   (= (position b3) shelf2)
   (= (contains b3) red-wine)
   (= (position b4) shelf1)
   (= (contains b4) lemonade)
   (= (position b5) shelf1)
   (= (contains b5) white-wine)
   (= (position b6) shelf3)
   (= (contains b6) water)
   (= (position b7) shelf1)
   (= (contains b7) orange-juice)
   (= (position b8) couch-table)
   (= (contains b8) red-wine)
   (= (position b9) dining-table)
   (= (contains b9) orange-juice)
   (= (position b10) shelf1)
   (= (contains b10) apple-juice)
   (= (position b11) shelf2)
   (= (contains b11) apple-juice)
   (= (position v0) dining-table)
   (= (contains v0) empty)
   (= (colored v0) red)
   (= (position v1) couch-table)
   (= (contains v1) empty)
   (= (colored v1) green)
   (= (colored f0) yellow)
   (= (position f0) flower-bed)
   (= (colored f1) yellow)
   (= (position f1) flower-bed)
   (= (colored f2) red)
   (= (position f2) flower-bed)
   (= (colored f3) red)
   (= (position f3) flower-bed)
 )

 (:goal (and)
)
)