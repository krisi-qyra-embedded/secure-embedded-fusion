/* Author: i fiksuari pas prog | Documentation date: 13 April 2026 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "vigenere_cipher.h"

/* Decrypts the supplied ciphertext into a data buffer. */
void decrypt_vigenere_cipher(char *encryptedtext, char *keyword, char *decryptedtext)
{
	int i,j = 0;
	/*Get keyword length*/
	int keyword_length =  strlen(keyword);

	/*Loop through each character in the encryptedtext*/
	for(i = 0; encryptedtext[i] != '\0'; i++ )
	{
		/*Reset keyword index if end is reached*/
			if( j == keyword_length)
			{
				j  = 0;
			}

			/*Check if the character is an uppercase letter*/
			if(encryptedtext[i] >= 'A'  && encryptedtext[i] <= 'Z')
			{
				/*Decrypt the character*/
				decryptedtext[i] = ((encryptedtext[i] - 'A' -(keyword[j] - 'A') + 26 )% 26) +'A';
				j++;
			}

			/*Check if the character is an uppercase letter*/
			else if(encryptedtext[i] >= 'a'  && encryptedtext[i] <= 'z')
			{
				/*Decrypt the character*/
				decryptedtext[i] = ((encryptedtext[i] - 'a' -(keyword[j] - 'a') + 26 )% 26) +'a';
				j++;
			}
			else{
				decryptedtext[i] = encryptedtext[i];
			}
	}

	decryptedtext[strlen(encryptedtext)] = '\0';
}


/* Encrypts the supplied data using the configured key. */
void encrypt_vigenere_cipher(char *plaintext, char *keyword, char *encryptedtext)
{
	int i,j = 0;
	/*Get keyword length*/
	int keyword_length =  strlen(keyword);

	/*Loop through each character in the plaintext*/
	for(i = 0; plaintext[i] != '\0'; i++ )
	{
		/*Reset keyword index if end is reached*/
		if( j == keyword_length)
		{
			j  = 0;
		}
		/*Check if the character is an uppercase letter*/
		if(plaintext[i] >= 'A'  && plaintext[i] <= 'Z')
		{
			/*Encrypt character*/
			encryptedtext[i] =  ((plaintext[i] - 'A' + (keyword[j] - 'A')) %26) + 'A';
			j++;
		}

		/*Check if the character is an lowercase letter*/
		else if(plaintext[i] >= 'a'  && plaintext[i] <= 'z')
		{
			/*Encrypt character*/
			encryptedtext[i] =  ((plaintext[i] - 'a' + (keyword[j] - 'a')) %26) + 'a';
			j++;
		}
		else{
			encryptedtext[i] = plaintext[i];
		}

	}
	encryptedtext[strlen(plaintext)] ='\0';
}


/* Prepares or updates cryptographic key material. */
void normalize_keyword(char *keyword)
{
    /* Validate the key data before using it in crypto operations. */
	for(int i = 0; keyword[i] != '\0'; i++)
	{
		keyword[i] =  toupper(keyword[i]);
	}
}



/* Encrypts the supplied data using the configured key. */
void encrypt_vigenere_cipher2(char *plaintext, char *keyword, char *encryptedtext)
{

	char c;
	int i,j = 0;
	/*Get keyword length*/
	int keyword_length =  strlen(keyword);

	/*Convert keyword to uppercase*/
	normalize_keyword(keyword);

	/*Loop through each character in the plaintext*/
	for(i = 0; plaintext[i] != '\0'; i++ )
	{
		/*Reset keyword index if end is reached*/
		if( j == keyword_length)
		{
			j  = 0;
		}

		 c = toupper(plaintext[i]);

		if( c >= 'A'  && c <= 'Z')
		{
			/*Encrypt character*/
			encryptedtext[i] =  ((c- 'A' + (keyword[j] - 'A')) %26) + 'A';
			j++;
		}

		else{
			encryptedtext[i] = plaintext[i];
		}

	}
	encryptedtext[strlen(plaintext)] ='\0';
}


/* Decrypts the supplied ciphertext into a data buffer. */
void decrypt_vigenere_cipher2(char *encryptedtext, char *keyword, char *decryptedtext)
{
	 char c;
	int i,j = 0;
	/*Get keyword length*/
	int keyword_length =  strlen(keyword);


	/*Convert keyword to uppercase*/
	normalize_keyword(keyword);

	/*Loop through each character in the encryptedtext*/
	for(i = 0; encryptedtext[i] != '\0'; i++ )
	{
		/*Reset keyword index if end is reached*/
			if( j == keyword_length)
			{
				j  = 0;
			}

			 c = toupper(encryptedtext[i]);

			/*Check if the character is an uppercase letter*/
			if(c >= 'A'  && c <= 'Z')
			{
				/*Decrypt the character*/
				decryptedtext[i] = ((c - 'A' -(keyword[j] - 'A') + 26 )% 26) +'A';
				j++;
			}
			else{
				decryptedtext[i] = encryptedtext[i];
			}
	}

	decryptedtext[strlen(encryptedtext)] = '\0';
}
