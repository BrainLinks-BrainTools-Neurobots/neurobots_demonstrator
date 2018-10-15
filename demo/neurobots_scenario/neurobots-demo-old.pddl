(define (problem neurobots)
  (:domain neurobots-demo)

  (:objects 
   omnirob iiwa - robot
   kitchen living-room pantry - room
   shelf dining-table couch-table - furniture
   me - human
   water lemonade orange-juice apple-juice - content  ;; empty is defined as special content in the domain file
   b1 b2 b3 b4 b5 b6 - bottle
   g1 g2 - cup
   )

  (:init
   (connected kitchen living-room)
   (connected living-room kitchen)
   (connected kitchen pantry)
   (connected pantry kitchen)

   (= (in shelf) pantry)
   (= (in dining-table) kitchen)
   (= (in couch-table) living-room)

   (= (in me) kitchen)

   (= (in iiwa) kitchen)
   (= (at iiwa) dining-table)
   (arm-empty iiwa)

   (= (in omnirob) pantry)
   (= (at omnirob) nowhere)
   (mobile omnirob)
   (arm-empty omnirob)

   (= (position b1) shelf)
   (= (contains b1) water)

   (= (position b2) shelf)
   (= (contains b2) lemonade)

   (= (position b3) shelf)
   (= (contains b3) orange-juice)

   (= (position b4) shelf)
   (= (contains b4) apple-juice)

   (= (position b5) couch-table)
   (= (contains b5) water)

   (= (position b6) shelf)
   (= (contains b6) water)

   (= (position g1) shelf)
   (is-open g1)
   (= (contains g1) empty)

   (= (position g2) couch-table) 
   (= (contains g2) lemonade)
   (is-open g2)
 
   )

  (:goal (and)
         )

  )


