(define (problem iros1)
  (:domain neurobots-iros1)

  (:objects 
   omnirob - robot
   table1 - table
   wheelchair - human
   shelfleft shelfright - furniture
   water - content  ;; empty is defined as special content in the domain file
   )

  (:init
  
   (= (at omnirob) nowhere)
   (mobile omnirob)
   (arm-empty omnirob)

   )

  (:goal (and)
         )

  )