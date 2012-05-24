/*******************************************************************************

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   FILENAME
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include "tcpip_private.h"

#if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
#include "tcpip/snmpv3.h"
#include "crypto/aes.h"

uint8_t md5LocalizedAuthKey[16];
uint8_t sha1LocalizedAuthKey[20];
uint8_t hmacAuthKeyBuf[64];
uint8_t authKey_iPad[64];
uint8_t authKey_oPad[64];
uint8_t HmacMd5Digest[16];
uint8_t HmacSHADigest[20];

uint8_t ivEncrptKeyOut[16];
uint8_t deciphered_text[16];
uint8_t cipher_text[16];
uint8_t snmpV3AesEncryptInitVector[16];//128 Bit
uint8_t snmpV3AesDecryptInitVector[16];//128 Bit
snmpV3EngnUserDataBase snmpV3UserDataBase[SNMPV3_USM_MAX_USER];


	
AES_SESSION_KEY_128_BIT    session_key;

extern uint8_t snmpInMsgAuthParamStrng[];
extern uint8_t snmpOutMsgAuthParamStrng[];
extern uint8_t snmpOutMsgPrvParamStrng[];
extern uint8_t snmpInMsgPrvParamStrng[];
extern uint8_t snmpOutMsgAuthParamLen;
extern uint8_t snmpInMsgAuthParamLen;
extern TCPIP_UINT32_VAL authoritativeSnmpEngineBoots;
extern TCPIP_UINT32_VAL authoritativeSnmpEngineTime;
extern uint16_t gSnmpv3UserDBIndex; 

extern SNMPV3MSGDATA gSNMPv3TrapMsgHeaderBuf;

void Snmpv3Pswd2LocalizedAuthKeyMD5Hashing(uint8_t* pswdToLocalized, uint8_t pswdLen);
void Snmpv3Pswd2LocalizedAuthKeySHAHashing(uint8_t* pswdToLocalized, uint8_t pswdLen);
void Snmpv3UsmSnmpEngnAuthPrivPswdLocalization(uint8_t userDBIndex);
void Snmpv3InitializeUserDataBase(void);
void Snmpv3ComputeMd5HmacCode(uint8_t xx_bits,uint8_t* digestptr,uint8_t * indata, uint32_t dataLen,
					  uint8_t* userExtendedLclzdKeyIpad, uint8_t* userExtendedLclzdKeyOpad);
void Snmpv3ComputeShaHmacCode(uint8_t xx_bits,uint8_t* digestptr, uint8_t * indata, uint32_t dataLen,
					 uint8_t* userExtendedLclzdKeyIpad,uint8_t* userExtendedLclzdKeyOpad);
void Snmpv3AuthKeyZeroing2HmacBufLen64(uint8_t* authKey, uint8_t authKeyLen,	uint8_t hashType);
uint8_t* Snmpv3ComputeHmacMD5Digest(uint8_t * inData, uint32_t dataLen,uint8_t* userExtendedLclzdKeyIpad,uint8_t* userExtendedLclzdKeyOpad);
uint8_t* Snmpv3ComputeHmacShaDigest(uint8_t * inData, uint32_t dataLen,uint8_t* userExtendedLclzdKeyIpad,uint8_t* userExtendedLclzdKeyOpad);

extern void Snmpv3AuthKeyZeroing2HmacBufLen64(uint8_t* authKey, uint8_t authKeyLen,  uint8_t hashType);




void Snmpv3InitializeUserDataBase(void)
{
uint8_t userDBIndex=0;
		memset(snmpV3UserDataBase,0,sizeof(snmpV3UserDataBase));

		memcpy(snmpV3UserDataBase[userDBIndex].userName,"microchip",strlen("microchip"));
		snmpV3UserDataBase[userDBIndex].userNameLength=strlen("microchip");
		memcpy(snmpV3UserDataBase[userDBIndex].userAuthPswd,"auth12345",strlen("auth12345"));
		snmpV3UserDataBase[userDBIndex].userAuthPswdLen=0x09;
		memcpy(snmpV3UserDataBase[userDBIndex].userPrivPswd,"priv12345",strlen("priv12345"));
		snmpV3UserDataBase[userDBIndex].userPrivPswdLen=0x09;
		snmpV3UserDataBase[userDBIndex].userHashType=SNMPV3_HAMC_MD5;
		snmpV3UserDataBase[userDBIndex].userPrivType=SNMPV3_AES_PRIV;
		snmpV3UserDataBase[userDBIndex].userDBIndex=userDBIndex;

		userDBIndex+=1;
		memcpy(snmpV3UserDataBase[userDBIndex].userName,"SnmpAdmin",strlen("SnmpAdmin"));
		snmpV3UserDataBase[userDBIndex].userNameLength=strlen("SnmpAdmin");
		memcpy(snmpV3UserDataBase[userDBIndex].userAuthPswd,"ChandlerUS",strlen("ChandlerUS"));
		snmpV3UserDataBase[userDBIndex].userAuthPswdLen=0x0a;
		memcpy(snmpV3UserDataBase[userDBIndex].userPrivPswd,"arizonaUSA",strlen("arizonaUSA"));
		snmpV3UserDataBase[userDBIndex].userPrivPswdLen=0x0a;
		snmpV3UserDataBase[userDBIndex].userHashType=SNMPV3_HMAC_SHA1;
		snmpV3UserDataBase[userDBIndex].userPrivType=SNMPV3_NO_PRIV;
		snmpV3UserDataBase[userDBIndex].userDBIndex=userDBIndex;
		
		userDBIndex+=1;
		memcpy(snmpV3UserDataBase[userDBIndex].userName,"root",strlen("root"));
		snmpV3UserDataBase[userDBIndex].userNameLength=strlen("root");
		memcpy(snmpV3UserDataBase[userDBIndex].userAuthPswd,"authAdmin",strlen("authAdmin"));
		snmpV3UserDataBase[userDBIndex].userAuthPswdLen=0x09;
		memcpy(snmpV3UserDataBase[userDBIndex].userPrivPswd,"privAdmin",strlen("privAdmin"));
		snmpV3UserDataBase[userDBIndex].userPrivPswdLen=0x09;
		snmpV3UserDataBase[userDBIndex].userHashType=SNMPV3_NO_HMAC_AUTH;
		snmpV3UserDataBase[userDBIndex].userPrivType=SNMPV3_NO_PRIV;
		snmpV3UserDataBase[userDBIndex].userDBIndex=userDBIndex;

}


//RFC3826
void Snmpv3UsmAesEncryptDecryptInitVector(uint8_t inOutPdu)
{
	uint8_t j;
	uint8_t* decryptPtr;
	uint8_t* prvParamPtr;
	
	if(inOutPdu== SNMP_REQUEST_PDU) //init vector for decryption
	{

		prvParamPtr=snmpInMsgPrvParamStrng;
		decryptPtr= snmpV3AesDecryptInitVector;
		
		//RFC 3826 section 3.1.2.1 Page #7
		//snmpV3AesDecryptInitVector=AuthSnmpEngnBoots+AuthSnmpEngnTime+inMsgPrivParam;
		*decryptPtr++=authoritativeSnmpEngineBoots.v[3];
		*decryptPtr++=authoritativeSnmpEngineBoots.v[2];
		*decryptPtr++=authoritativeSnmpEngineBoots.v[1];
		*decryptPtr++=authoritativeSnmpEngineBoots.v[0];

		*decryptPtr++=authoritativeSnmpEngineTime.v[3];
		*decryptPtr++=authoritativeSnmpEngineTime.v[2];
		*decryptPtr++=authoritativeSnmpEngineTime.v[1];
		*decryptPtr++=authoritativeSnmpEngineTime.v[0];

		j=0;
		while(1)
		{
			*decryptPtr++=*(prvParamPtr+j);
			if(j==7)
				break;
			j++;
		}	

	}
	else if(inOutPdu == SNMP_RESPONSE_PDU) //init vector for encryption
	{

		prvParamPtr=snmpOutMsgPrvParamStrng;
		decryptPtr= snmpV3AesEncryptInitVector;
		
		*decryptPtr++=authoritativeSnmpEngineBoots.v[3];
		*decryptPtr++=authoritativeSnmpEngineBoots.v[2];
		*decryptPtr++=authoritativeSnmpEngineBoots.v[1];
		*decryptPtr++=authoritativeSnmpEngineBoots.v[0];

		*decryptPtr++=snmpEngineTime.v[3];
		*decryptPtr++=snmpEngineTime.v[2];
		*decryptPtr++=snmpEngineTime.v[1];
		*decryptPtr++=snmpEngineTime.v[0];

		j=0;
		while(1)
		{
			*decryptPtr++=*(prvParamPtr+j);
			if(j==7)
				break;
			j++;
		}

	}
}

void Snmpv3USMOutMsgPrivParam(void)
{
	uint8_t* prvParamPtr;

	//SNMP ENgine Time is 32 Bit counter. 64 Bit counter (Extended Data type) to be implemented.
	Snmpv3GetAuthEngineTime();

	prvParamPtr=snmpOutMsgPrvParamStrng;	
	
	*prvParamPtr++=0x00;
	*prvParamPtr++=0x00;
	*prvParamPtr++=0x00;
	*prvParamPtr++=0x00;

	//ORing is done to generate some random number from the snmp engine time rand() not used. 
	*prvParamPtr++=snmpEngineTime.v[3]^0xFF;
	*prvParamPtr++=snmpEngineTime.v[2]^0xEE;
	*prvParamPtr++=snmpEngineTime.v[1]^0xDD;
	*prvParamPtr++=snmpEngineTime.v[0]^0xCC;
	
}


void Snmpv3UsmOutMsgAuthenticationParam(uint8_t hashType)
{
	if(hashType == SNMPV3_HAMC_MD5)
	{
		Snmpv3ComputeMd5HmacCode(96,snmpOutMsgAuthParamStrng,(uint8_t*)gSNMPv3TrapMsgHeaderBuf.head,
			gSNMPv3TrapMsgHeaderBuf.length,
				snmpV3UserDataBase[0].userAuthLocalKeyHmacIpad,snmpV3UserDataBase[0].userAuthLocalKeyHmacOpad);
	}
	else if(hashType == SNMPV3_HMAC_SHA1)
	{
		Snmpv3ComputeShaHmacCode(96,snmpOutMsgAuthParamStrng,(uint8_t*)gSNMPv3TrapMsgHeaderBuf.head,
			gSNMPv3TrapMsgHeaderBuf.length,
				snmpV3UserDataBase[0].userAuthLocalKeyHmacIpad,snmpV3UserDataBase[0].userAuthLocalKeyHmacOpad);

	}
}



void Snmpv3UsmSnmpEngnAuthPrivPswdLocalization(uint8_t userDBIndex)
{
	if(snmpV3UserDataBase[userDBIndex].userHashType== SNMPV3_HAMC_MD5)
	{
		Snmpv3Pswd2LocalizedAuthKeyMD5Hashing(snmpV3UserDataBase[userDBIndex].userAuthPswd,
											snmpV3UserDataBase[userDBIndex].userAuthPswdLen);
		memcpy(snmpV3UserDataBase[userDBIndex].userAuthPswdLoclizdKey,md5LocalizedAuthKey,16);

		if((snmpV3UserDataBase[userDBIndex].userPrivPswdLen != 0x00)&& 
	   (snmpV3UserDataBase[userDBIndex].userPrivType == SNMPV3_AES_PRIV))
	{
		Snmpv3Pswd2LocalizedAuthKeyMD5Hashing(snmpV3UserDataBase[userDBIndex].userPrivPswd,
											snmpV3UserDataBase[userDBIndex].userPrivPswdLen);

		memcpy(snmpV3UserDataBase[userDBIndex].userPrivPswdLoclizdKey,md5LocalizedAuthKey,16);	
	}
	}
	else if(snmpV3UserDataBase[userDBIndex].userHashType == SNMPV3_HMAC_SHA1)
	{
		Snmpv3Pswd2LocalizedAuthKeySHAHashing(snmpV3UserDataBase[userDBIndex].userAuthPswd,
											snmpV3UserDataBase[userDBIndex].userAuthPswdLen);
		memcpy(snmpV3UserDataBase[userDBIndex].userAuthPswdLoclizdKey,sha1LocalizedAuthKey,20);

		if((snmpV3UserDataBase[userDBIndex].userPrivPswdLen != 0x00)&& 
	   (snmpV3UserDataBase[userDBIndex].userPrivType == SNMPV3_AES_PRIV))
	{
		Snmpv3Pswd2LocalizedAuthKeySHAHashing(snmpV3UserDataBase[userDBIndex].userPrivPswd,
											snmpV3UserDataBase[userDBIndex].userPrivPswdLen);

		memcpy(snmpV3UserDataBase[userDBIndex].userPrivPswdLoclizdKey,sha1LocalizedAuthKey,20);	
	}
	}

	/*if((snmpV3UserDataBase[userDBIndex].userPrivPswdLen != 0x00)&& 
	   (snmpV3UserDataBase[userDBIndex].userPrivType == SNMPV3_AES_PRIV))
	{
		Snmpv3Pswd2LocalizedAuthKeyMD5Hashing(snmpV3UserDataBase[userDBIndex].userPrivPswd,
											snmpV3UserDataBase[userDBIndex].userPrivPswdLen);

		memcpy(snmpV3UserDataBase[userDBIndex].userPrivPswdLoclizdKey,md5LocalizedAuthKey,16);	
	}*/
	return;
	
}



void Snmpv3Pswd2LocalizedAuthKeyMD5Hashing(uint8_t* pswdToLocalized, uint8_t pswdLen)
{
static HASH_SUM md5;
uint8_t *compressionPtr, pswdBuf[64];
uint32_t index = 0;
uint32_t count = 0, i;
uint8_t* pswdPtr;

	pswdPtr=pswdToLocalized;

	MD5Initialize(&md5);
	
	while (count < 1048576)
	{
		compressionPtr = pswdBuf;
		for (i = 0; i < 64; i++) 
		{
			*compressionPtr++ = pswdPtr[index++ % pswdLen];
		}


		MD5AddData(&md5, pswdBuf, 64);
		count+=64;
	
	}
	MD5Calculate(&md5, md5LocalizedAuthKey);

	memcpy(pswdBuf, md5LocalizedAuthKey, 16 /*localizedAuthKey buf len*/);
	memcpy(pswdBuf+16, snmpEngineID, snmpEngnIDLength);
	memcpy(pswdBuf+16+snmpEngnIDLength, md5LocalizedAuthKey, 16 /*localizedAuthKey buf len*/);

	MD5Initialize(&md5);
	MD5AddData(&md5,pswdBuf,32+snmpEngnIDLength);
	
	MD5Calculate(&md5, md5LocalizedAuthKey);

	count+=64;

	return;
}

void Snmpv3Pswd2LocalizedAuthKeySHAHashing(uint8_t* pswdToLocalized, uint8_t pswdLen)
{
static HASH_SUM sha1;
uint8_t *compressionPtr, pswdBuf[72];
uint32_t index = 0;
uint32_t count = 0, i;
uint8_t* pswdPtr;

	pswdPtr=pswdToLocalized;

	SHA1Initialize(&sha1);
	
	while (count < 1048576)
	{
		compressionPtr = pswdBuf;
		for (i = 0; i < 64; i++) 
		{
			*compressionPtr++ = pswdPtr[index++ % pswdLen];
		}


		SHA1AddData(&sha1, pswdBuf, 64);
		count+=64;
	
	}
	SHA1Calculate(&sha1, sha1LocalizedAuthKey);

	memcpy(pswdBuf, sha1LocalizedAuthKey, 20 /*SHA1 localizedAuthKey buf len*/);
	memcpy(pswdBuf+20, snmpEngineID, snmpEngnIDLength);
	memcpy(pswdBuf+20+snmpEngnIDLength, sha1LocalizedAuthKey, 20 /*SHA1 localizedAuthKey buf len*/);

	SHA1Initialize(&sha1);
	SHA1AddData(&sha1,pswdBuf,40+snmpEngnIDLength);
	
	SHA1Calculate(&sha1, sha1LocalizedAuthKey);

	count+=64;

	return;
}

void Snmpv3ComputeHMACIpadOpadForAuthLoclzedKey(uint8_t userDBIndex)
{
	if(snmpV3UserDataBase[userDBIndex].userHashType==SNMPV3_HAMC_MD5)
	{
		Snmpv3AuthKeyZeroing2HmacBufLen64(snmpV3UserDataBase[userDBIndex].userAuthPswdLoclizdKey,
								16,snmpV3UserDataBase[userDBIndex].userHashType);
	}
	else if(snmpV3UserDataBase[userDBIndex].userHashType==SNMPV3_HMAC_SHA1)
	{
		Snmpv3AuthKeyZeroing2HmacBufLen64(snmpV3UserDataBase[userDBIndex].userAuthPswdLoclizdKey,
								20,snmpV3UserDataBase[userDBIndex].userHashType);
	}
	//authKeyInnerOuterPadding();

	//Authorazation key inner padding
	uint8_t i=0;
	for(i=0;i<64;i++)
		authKey_iPad[i]=hmacAuthKeyBuf[i]^0x36;

	//Authorazation key outer padding
	for(i=0;i<64;i++)
		authKey_oPad[i]=hmacAuthKeyBuf[i]^0x5c;

	memcpy(snmpV3UserDataBase[userDBIndex].userAuthLocalKeyHmacIpad,authKey_iPad,64);
	memcpy(snmpV3UserDataBase[userDBIndex].userAuthLocalKeyHmacOpad,authKey_oPad,64);
}






uint8_t Snmpv3AuthenticateRxedPduForDataIntegrity(SNMPV3_REQUEST_WHOLEMSG* rxDataPtr)
{	
uint8_t reportMsgName[7]="initial";//respose is "report" 0xa8 msg
uint8_t* secNamePtr;
uint8_t i;
uint16_t authParamOffset;
uint8_t hashTYpe;
static HASH_SUM md5;
uint8_t* tempPtr;

	secNamePtr= securityPrimitivesOfIncomingPdu.securityName;
	hashTYpe=snmpV3UserDataBase[gSnmpv3UserDBIndex].userHashType;

	//Check if the received packet is expecting "report" as response.
	if(!strncmp((const char *)secNamePtr,
				(const char *)reportMsgName,		
				(securityPrimitivesOfIncomingPdu.securityNameLength)))
		return false; //If "report" is expected, Retrun. 

	authParamOffset=gSnmpV3InPduWholeMsgBuf.msgAuthParamOffsetInWholeMsg;

	tempPtr=gSnmpV3InPduWholeMsgBuf.snmpMsgHead;
	for(i=0;i<snmpInMsgAuthParamLen /*Should be 12 Bytes*/;i++)
	{

		//RFC3414 Section 6.3.2 Page#56 Step3
		*(tempPtr+authParamOffset+i)=0x00;
	}

	if(hashTYpe == SNMPV3_HAMC_MD5)
	{

		MD5Initialize(&md5);
		MD5AddData(&md5,snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacIpad, (uint16_t)0x40);
		MD5AddData(&md5, rxDataPtr->wholeMsgHead, rxDataPtr->wholeMsgLen.Val);
		MD5Calculate(&md5, HmacMd5Digest);
		
		MD5Initialize(&md5);
		MD5AddData(&md5, snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacOpad, (uint16_t)0x40);
		MD5AddData(&md5, HmacMd5Digest,16);
		MD5Calculate(&md5, HmacMd5Digest);

		
	}
	else if(hashTYpe == SNMPV3_HMAC_SHA1)
	{
		SHA1Initialize(&md5);
		SHA1AddData(&md5,snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacIpad, (uint16_t)0x40);
		SHA1AddData(&md5, rxDataPtr->wholeMsgHead, rxDataPtr->wholeMsgLen.Val);
		SHA1Calculate(&md5, HmacSHADigest);
		
		SHA1Initialize(&md5);
		SHA1AddData(&md5, snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacOpad, (uint16_t)0x40);
		SHA1AddData(&md5, HmacSHADigest,20);
		SHA1Calculate(&md5, HmacSHADigest);
		//return true;
	}
	else
		return SNMPV3_MSG_AUTH_FAIL ;

	if(hashTYpe == SNMPV3_HAMC_MD5)
	{
		i=strncmp((const char *)&snmpInMsgAuthParamStrng,(const char *)&HmacMd5Digest,12);
	}
	else if(hashTYpe == SNMPV3_HMAC_SHA1)
	{
		i=strncmp((const char *)&snmpInMsgAuthParamStrng,(const char *)&HmacSHADigest,12);
	}	
	if(i!=0)
		return SNMPV3_MSG_AUTH_FAIL;

	
	//Authparam validated on WholeMsg. Write back the auth param string to received buffer	
	tempPtr=gSnmpV3InPduWholeMsgBuf.snmpMsgHead;
	for(i=0;i<snmpInMsgAuthParamLen /*Should be 12 Bytes*/;i++)
	{
		*(tempPtr+authParamOffset+i)=snmpInMsgAuthParamStrng[i];
	}

	return SNMPV3_MSG_AUTH_PASS;
		

}


uint8_t Snmpv3AuthenticateTxPduForDataIntegrity(SNMPV3_RESPONSE_WHOLEMSG* txDataPtr)
{	
uint8_t* secNamePtr;
uint8_t i;
static HASH_SUM md5;
uint8_t* tempPtr;
uint8_t hashTYpe;

	hashTYpe=snmpV3UserDataBase[gSnmpv3UserDBIndex].userHashType;	

	if(hashTYpe == SNMPV3_HAMC_MD5)
	{

		MD5Initialize(&md5);
		MD5AddData(&md5,snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacIpad, (uint16_t)0x40);
		MD5AddData(&md5, txDataPtr->wholeMsgHead, txDataPtr->wholeMsgLen.Val);
		MD5Calculate(&md5, HmacMd5Digest);
		
		MD5Initialize(&md5);
		MD5AddData(&md5, snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacOpad, (uint16_t)0x40);
		MD5AddData(&md5, HmacMd5Digest,16);
		MD5Calculate(&md5, HmacMd5Digest);

		
	}
	else if(hashTYpe == SNMPV3_HMAC_SHA1)
	{
		SHA1Initialize(&md5);
		SHA1AddData(&md5,snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacIpad, (uint16_t)0x40);
		SHA1AddData(&md5, txDataPtr->wholeMsgHead, txDataPtr->wholeMsgLen.Val);
		SHA1Calculate(&md5, HmacSHADigest);
		
		SHA1Initialize(&md5);
		SHA1AddData(&md5, snmpV3UserDataBase[gSnmpv3UserDBIndex].userAuthLocalKeyHmacOpad, (uint16_t)0x40);
		SHA1AddData(&md5, HmacSHADigest,20);
		SHA1Calculate(&md5, HmacSHADigest);
	
	}
	else
		return SNMPV3_MSG_AUTH_FAIL ;

	//Authparam validated on WholeMsg. Write back the auth param string to received buffer	
	tempPtr=snmpOutMsgAuthParamStrng;
	if(hashTYpe == SNMPV3_HAMC_MD5)
		secNamePtr=HmacMd5Digest;
	else if(hashTYpe == SNMPV3_HMAC_SHA1)
		secNamePtr=HmacSHADigest;
		

	i=0;
	for(i=0;i < 12/*snmpOutMsgAuthParamLen Should be 12 Bytes*/;i++)
	{
		tempPtr[i]=secNamePtr[i];
	}
	return SNMPV3_MSG_AUTH_PASS;
}


uint8_t Snmpv3AESDecryptRxedScopedPdu(/*uint8_t userDBIndex*/)
{

uint8_t* cryptoKey;
uint8_t* initVector;
uint8_t* cipher_text;
uint16_t cipherTextLen;
uint8_t* decrypted_text;
uint16_t temp;
uint8_t extraMemReqd;

	AES_ROUND_KEYS_128_BIT round_keys;
	AES_CFB_STATE_DATA current_stream;

	cryptoKey=snmpV3UserDataBase[gSnmpv3UserDBIndex].userPrivPswdLoclizdKey;
	initVector=snmpV3AesDecryptInitVector;
	temp=gSnmpV3InPduWholeMsgBuf.scopedPduOffset;
	cipher_text=(gSnmpV3InPduWholeMsgBuf.snmpMsgHead+temp);
	cipherTextLen= gSnmpV3InPduWholeMsgBuf.scopedPduStructLen;

	extraMemReqd=(16-(cipherTextLen%16)); //AES Blocks are in multiples of 16 Bytes
	decrypted_text=(uint8_t*)(malloc((size_t)cipherTextLen+extraMemReqd));
	
	if(decrypted_text != NULL)
	{
				
		AESCreateRoundKeys (&round_keys,cryptoKey,AES_KEY_SIZE_128_BIT);
	
		memcpy(current_stream.initial_vector,initVector,16);
	
		AESCFBDecrypt(decrypted_text,cipher_text, cipherTextLen,	
						&round_keys, &current_stream,		 
						AES_STREAM_START | AES_USE_CFB128);
	}
	else
		return SNMPV3_MSG_PRIV_FAIL;

	//Copy decrypted text to already allocated WholeMsg dynamic memory Buffer.
	memcpy(cipher_text,decrypted_text,cipherTextLen);

	//free this temp buffer used for decryption purpose.
	free(decrypted_text);

	return SNMPV3_MSG_PRIV_PASS;
}


uint8_t Snmpv3AESEncryptResponseScopedPdu(SNMPV3_RESPONSE_WHOLEMSG* plain_text/*uint8_t userDBIndex*/)
{

uint8_t* cryptoKey;
uint8_t* initVector;
uint8_t* plainText;
uint16_t plaintextLen;
uint8_t* encrypted_text;
uint8_t extraMemReqd;
AES_ROUND_KEYS_128_BIT round_keys;
AES_CFB_STATE_DATA current_stream;

		
	//This is a secured request. Compute the AES Encryption IV 
	Snmpv3UsmAesEncryptDecryptInitVector(SNMP_RESPONSE_PDU);

	plaintextLen= (plain_text->scopedPduStructLen);
	cryptoKey=snmpV3UserDataBase[gSnmpv3UserDBIndex].userPrivPswdLoclizdKey;
	initVector=snmpV3AesEncryptInitVector;
	plainText=(plain_text->scopedPduOffset);
	

	extraMemReqd=(16-(plaintextLen%16)); //AES Blocks are in multiples of 16 Bytes
	encrypted_text=(uint8_t*)(malloc((size_t)plaintextLen+extraMemReqd));

	if(encrypted_text != NULL)
	{		
		AESCreateRoundKeys (&round_keys,cryptoKey,AES_KEY_SIZE_128_BIT);

        memcpy(current_stream.initial_vector,initVector,16);

		AESCFBEncrypt(encrypted_text,plainText, plaintextLen,    
                    &round_keys, &current_stream,        
                    AES_STREAM_START | AES_USE_CFB128 );

	}
	else
		return	SNMPV3_MSG_PRIV_FAIL;

	//Copy decrypted text to already allocated WholeMsg dynamic memory Buffer.
	memcpy(plainText,encrypted_text,plaintextLen);

	//free this temp buffer used for decryption purpose.
	free(encrypted_text);

	return SNMPV3_MSG_PRIV_PASS;
}


bool Snmpv3ValidateEngineId(void)
{
uint8_t* inEngnIdPtr=NULL;
uint8_t temp;
uint8_t reportMsgName[7]="initial";//respose is "report" 0xa8 msg
uint8_t* secNamePtr=NULL;

	secNamePtr= securityPrimitivesOfIncomingPdu.securityName;

	//Check if the received packet is expecting "report" as response.
	if(!strncmp((const char *)secNamePtr,
				(const char *)reportMsgName,		
				(securityPrimitivesOfIncomingPdu.securityNameLength)))
		return true; //If "report" is expected, Retrun. 

	else
	{
	
		inEngnIdPtr=securityPrimitivesOfIncomingPdu.securityEngineID;

		temp=strncmp((const char *)inEngnIdPtr,
					(const char *)snmpEngineID,		
					(securityPrimitivesOfIncomingPdu.securityEngineIDLen));
		if(temp!=0)
			return false; //If "report" is expected, Retrun. 
		else
			return true;
	}
}


bool Snmpv3ValidateSecurityName(void)
{
uint8_t* inSecNamePtr;
uint8_t tempLen,i,temp;
uint8_t reportMsgName[7]="initial";//respose is "report" 0xa8 msg

	tempLen=securityPrimitivesOfIncomingPdu.securityNameLength;
	inSecNamePtr=securityPrimitivesOfIncomingPdu.securityName;

	//Check if the received packet is expecting "report" as response.
	if(!strncmp((const char *)inSecNamePtr,
					(const char *)reportMsgName,		
					tempLen))
	{
			gSnmpv3UserDBIndex=0;
			return true; //If "report" is expected, Retrun. 
	}
	else
	{
		for(i=0;i<SNMPV3_USM_MAX_USER;i++)
		{
			temp=strncmp((const char *)snmpV3UserDataBase[i].userName,
						  (const char *)inSecNamePtr,tempLen);

			if(temp==0)
			{
				gSnmpv3UserDBIndex=i;
				return true;
			}
		}
		gSnmpv3UserDBIndex=0;
	}
	return false;

}

uint8_t Snmpv3GetSecurityLevel(uint8_t userIndex)
{

	if(((snmpV3UserDataBase[userIndex].userHashType == SNMPV3_HAMC_MD5) || (snmpV3UserDataBase[userIndex].userHashType == SNMPV3_HMAC_SHA1))
	&& ((snmpV3UserDataBase[userIndex].userPrivType == SNMPV3_AES_PRIV) || (snmpV3UserDataBase[userIndex].userPrivType == SNMPV3_DES_PRIV)))
		return NO_REPORT_PRIVACY_AND_AUTH_PROVIDED;
	else if(((snmpV3UserDataBase[userIndex].userHashType == SNMPV3_HAMC_MD5) || (snmpV3UserDataBase[userIndex].userHashType == SNMPV3_HMAC_SHA1))
		&& (snmpV3UserDataBase[userIndex].userPrivType == SNMPV3_NO_PRIV))
		return NO_REPORT_NO_PRIVACY_BUT_AUTH_PROVIDED;
	else
		return NO_REPORT_NO_PRIVACY_NO_AUTH;
	

}

bool Snmpv3ValidateSecNameAndSecLvl(void)
{
	uint8_t* inSecNamePtr=NULL;
	uint8_t reportMsgName[7]="initial";//respose is "report" 0xa8 msg
	uint8_t  tempLen=0,i=0,temp=0;
	uint8_t  inSecurityLevel=0;

	
	
	tempLen=securityPrimitivesOfIncomingPdu.securityNameLength;
	if(tempLen == 0x0u)
		return false;

	inSecNamePtr=securityPrimitivesOfIncomingPdu.securityName;
	inSecurityLevel = securityPrimitivesOfIncomingPdu.securityLevel;

	
	if(!strncmp((const char *)inSecNamePtr,
					(const char *)reportMsgName,		
					tempLen))
	{
			gSnmpv3UserDBIndex=0;
			return true; //If "report" is expected, Retrun. 
	}
	else
	{
		for(i=0;i<SNMPV3_USM_MAX_USER;i++)
		{
			if(tempLen != snmpV3UserDataBase[i].userNameLength)
				continue;
			temp=strncmp((const char *)snmpV3UserDataBase[i].userName,
						  (const char *)inSecNamePtr,snmpV3UserDataBase[i].userNameLength);

			if((temp==0) && (Snmpv3GetSecurityLevel(i) == (inSecurityLevel&0x03)))
			{
				gSnmpv3UserDBIndex=i;
				return true;
			}
		}
		gSnmpv3UserDBIndex=0;
	}
	return false;
}



void Snmpv3ComputeMd5HmacCode(uint8_t xx_bits,uint8_t* digestptr, 			
					  uint8_t * indata, uint32_t dataLen,
					  uint8_t* userExtendedLclzdKeyIpad,
					  uint8_t* userExtendedLclzdKeyOpad)
{
uint8_t* hmacMd5DigestPtr;
uint8_t i;
uint8_t* dataPtr;
dataPtr=indata;

	
	hmacMd5DigestPtr=Snmpv3ComputeHmacMD5Digest(dataPtr, dataLen,userExtendedLclzdKeyOpad,userExtendedLclzdKeyOpad);
	
	for(i=0;i<(xx_bits/8);i++)
	{
		digestptr[i]=*(hmacMd5DigestPtr+i);
	}

}


void Snmpv3ComputeShaHmacCode(uint8_t xx_bits,uint8_t* digestptr,
					uint8_t * indata, uint32_t dataLen,
					uint8_t* userExtendedLclzdKeyIpad,
					uint8_t* userExtendedLclzdKeyOpad)
{
uint8_t* hmacSHADigestPtr;
uint8_t i;
uint8_t* dataptr;
dataptr=indata;


	hmacSHADigestPtr=Snmpv3ComputeHmacShaDigest(dataptr, dataLen,userExtendedLclzdKeyOpad,userExtendedLclzdKeyOpad);

	for(i=0;i<(xx_bits/8);i++)
	{
		digestptr[i]=*(hmacSHADigestPtr+i);
	}

}



void Snmpv3AuthKeyZeroing2HmacBufLen64(uint8_t* authKey, uint8_t authKeyLen,  uint8_t hashType)
{
	uint8_t* tempAuthKeyptr;
	uint8_t i;

	
	tempAuthKeyptr = authKey;

	if(authKeyLen > 64)
	{
		if(hashType == SNMPV3_HAMC_MD5)
		{
			//Hash MD5 AuthKey;
			//Zero pad the Auth key;
		}
		else if(hashType == SNMPV3_HMAC_SHA1)
		{
			//Hash SHA AuthKey;
			//Zero pad the Auth key;
		}
	}
	else 
	{
		//ZeroPad Auth Key	
		memcpy((void*) &hmacAuthKeyBuf, (const void *)tempAuthKeyptr, authKeyLen);

		for(i=authKeyLen;i<64;i++)
			hmacAuthKeyBuf[i]=0x00;
	}

}




uint8_t* Snmpv3ComputeHmacMD5Digest(uint8_t * inData, uint32_t dataLen,uint8_t* userExtendedLclzdKeyIpad,uint8_t* userExtendedLclzdKeyOpad)
{
	static HASH_SUM md5;
	uint8_t * data2Hmac;
	
	data2Hmac=inData;
	
	MD5Initialize(&md5);
	MD5AddData(&md5, userExtendedLclzdKeyIpad, (uint16_t)0x40);
	MD5AddData(&md5, data2Hmac, (uint16_t)dataLen);
	MD5Calculate(&md5, HmacMd5Digest);


	
	MD5Initialize(&md5);
	MD5AddData(&md5, userExtendedLclzdKeyOpad, (uint16_t)0x40);
	MD5AddData(&md5, HmacMd5Digest,16);
	MD5Calculate(&md5, HmacMd5Digest);

	return HmacMd5Digest;


}



uint8_t* Snmpv3ComputeHmacShaDigest(uint8_t * inData, uint32_t dataLen,uint8_t* userExtendedLclzdKeyIpad,uint8_t* userExtendedLclzdKeyOpad)
{	
	
	static HASH_SUM sha1;
	uint8_t * data2Hmac;
	
	data2Hmac=inData;
	
	SHA1Initialize(&sha1);
	SHA1AddData(&sha1, authKey_iPad, (uint16_t)0x40);
	SHA1AddData(&sha1, data2Hmac, (uint16_t)dataLen);
	SHA1Calculate(&sha1, HmacSHADigest);


	
	SHA1Initialize(&sha1);
	SHA1AddData(&sha1, authKey_oPad, (uint16_t)0x40);
	SHA1AddData(&sha1, HmacSHADigest,20);
	SHA1Calculate(&sha1, HmacSHADigest);

	return HmacSHADigest;
	
}

#endif // #if defined(TCPIP_STACK_USE_SNMPV3_SERVER)
