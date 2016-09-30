
 
#define MAX_L 255
 char o_string[MAX_L]={'a','b'}, s_string[MAX_L]={'c'}, r_string[MAX_L];
 String s1;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

;

}

void loop() {
  // put your main code here, to run repeatedly:
  
  replace(o_string, s_string, r_string);
  Serial.println(s_string[0]);
  Serial.println(r_string[0]);
  Serial.println("cscsa");

}


/**
 ****************************************************|
 * String replace Program                            |
 ****************************************************|
 * Takes three string input from the user
 * Replaces all the occurances of the second string
 * with the third string from the first string
 * @author Swashata
 */


/** Define the max char length */


/** Prototypes */
//void replace (char *, char *, char *);

//int main(void) {
   // char o_string[MAX_L], s_string[MAX_L], r_string[MAX_L]; //String storing variables

  //printf("Please enter the original string (max length %d characters): ", MAX_L);
  

  //printf("\nPlease enter the string to search (max length %d characters): ", MAX_L);
  

  //printf("\nPlease enter the replace string (max length %d characters): ", MAX_L);
  

 // printf("\n\nThe Original string\n*************************************\n");
  


  //printf("\n\nThe replaced string\n*************************************\n");
  

    //return 0;
//}

/**
 * The replace function
 *
 * Searches all of the occurrences using recursion
 * and replaces with the given string
 * @param char * o_string The original string
 * @param char * s_string The string to search for
 * @param char * r_string The replace string
 * @return void The o_string passed is modified
 */
void replace(char * o_string, char * s_string, char * r_string) {
      //a buffer_str variable to do all replace things
      char buffer_str[MAX_L];
      //to store the pointer returned from strstr
      char * ch;

      //first exit condition
      if(!(ch = strstr(o_string, s_string)))
              return;

      //copy all the content to buffer_str before the first occurrence of the search string
      //strncpy(buffer_str, o_string, ch-o_string);
  strcpy(buffer_str, o_string);

      //prepare the buffer_str for appending by adding a null to the end of it
      buffer_str[ch-o_string] = 0;

      //append using sprintf function
      //sprintf(buffer_str+(ch - o_string), "%s%s", r_string, ch + strlen(s_string));
        s1=String(buffer_str+(ch-o_string));
        s1.toCharArray(r_string,s1.length());
      //empty o_string for copying
      o_string[0] = 0;
      strcpy(o_string, buffer_str);
      //pass recursively to replace other occurrences
      return replace(o_string, s_string, r_string);
 }
