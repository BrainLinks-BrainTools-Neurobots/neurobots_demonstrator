(define (problem neurobots)
  (:domain neurobots-demo)

  (:objects 
   omnirob iiwa - robot
   kitchen living-room - room
   shelf dining-table - furniture
   me - human
   water - content  ;; empty is defined as special content in the domain file
   waterbottle1 - bottle
   redcup1 - cup
   )

  (:init
   (connected kitchen living-room)
   (connected living-room kitchen)

   (= (in shelf) kitchen)
   (= (in dining-table) living-room)

   (= (in me) living-room)

   (= (in iiwa) living-room)
   (= (at iiwa) dining-table)
   (arm-empty iiwa)

   (= (in omnirob) kitchen)
   (= (at omnirob) nowhere)
   (mobile omnirob)
   (arm-empty omnirob)

   (= (position waterbottle1) dining-table)
   (= (contains waterbottle1) water)

   (= (position redcup1) dining-table)
   (is-open redcup1)
   (= (contains redcup1) empty)

   )

  (:goal (and)
         )

  )


