(define (problem iros2)
  (:domain neurobots-iros2)

  (:objects 
   omnirob - robot
   me friend - human
   shelfleft shelfright - shelf
   dining-table couch-table - table
   kitchen living-room pantry - room
   ballon wwineglas cylinder beerstein - shape
   water lemonade apple-juice orange-juice red-wine white-wine beer - content  ;; empty is defined as special content in the domain file
   b1 b2 b3 b4 b5 b6 b7 - bottle
   c1 c2 - cup
   g1 g2 g3 g4 g5 - glass
)

  (:init
   (connected kitchen living-room)
   (connected living-room kitchen)
   (connected kitchen pantry)
   (connected pantry kitchen)
   (connected kitchen kitchen)
   (connected living-room living-room)
   (connected pantry pantry)

   (= (in shelfleft) pantry)
   (= (in shelfright) pantry)
   (= (in dining-table) kitchen)
   (= (in couch-table) living-room)

   (= (in me) living-room)
   (= (in friend) living-room)

   (= (in omnirob) kitchen)
   (= (at omnirob) nowhere)
   (mobile omnirob)
   (arm-empty omnirob)

   (= (position b1) shelfleft)
   (= (contains b1) water)

   (= (position b2) shelfleft)
   (= (contains b2) lemonade)

   (= (position b3) couch-table)
   (= (contains b3) orange-juice)

   (= (position b4) shelfleft)
   (= (contains b4) apple-juice)

   (= (position b5) shelfleft)
   (= (contains b5) beer)

   (= (position b6) shelfright)
   (= (contains b6) white-wine)

   (= (position b7) shelfleft)
   (= (contains b7) red-wine)
   
   (= (position g1) shelfright)
   (is-open g1)
   (= (contains g1) empty)
   (= (shaped g1) ballon)

   (= (position g2) dining-table)
   (is-open g2)
   (= (contains g2) empty)
   (= (shaped g2) wwineglas)
   
   (= (position g3) dining-table)
   (is-open g3)
   (= (contains g3) empty)
   (= (shaped g3) cylinder)
   
   (= (position g4) shelfright)
   (is-open g4)
   (= (contains g4) empty)
   (= (shaped g4) beerstein)
   
   (= (position g5) shelfleft)
   (is-open g5)
   (= (contains g5) empty)
   (= (shaped g5) ballon)
   
   (= (position c1) couch-table)
   (is-open c1)
   (= (contains c1) water)
   
   (= (position c2) shelfright)
   (is-open c2)
   (= (contains c2) empty)

 
   )

  (:goal (and)
         )

  )
