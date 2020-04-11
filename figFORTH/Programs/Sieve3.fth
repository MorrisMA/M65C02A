: prime? ( n -- ? ) here + c@ 0= ;
: composite! ( n -- ) here + 1 swap c! ;
: 2dup ( n1 n2 -- n1 n2 n1 n2 )  over over ;
: sieve ( n -- )
  here over 0 fill
  2                     
  begin
    2dup dup 
    * >
  while                     
    dup prime?
      if  2dup dup *                     
        do  i composite! dup
        +loop
      then
      1+
  repeat drop ." Primes: "
  2
  do i prime?
    if i . then
  loop ;
1000 sieve

