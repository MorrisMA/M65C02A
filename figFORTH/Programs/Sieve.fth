: 2DROP drop drop ;
16384 2 / CONSTANT maxp 
: SIEVE                             ( -- n )
  HERE maxp 1 FILL
  1                                 ( count, including 2 )
  maxp 0 DO
    I HERE + C@ IF
      I 2 * 3 + ( dup . ) DUP  I +  ( prime current )
      BEGIN  DUP maxp U<
      WHILE  0 OVER HERE + C!
             OVER +
      REPEAT
      2DROP 1+
    THEN
  LOOP ;
: PRIMES ." S" 10 1 do sieve sp! loop sp! ." E" ;

  