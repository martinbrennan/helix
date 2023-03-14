#include "string.h"
#include "sha1.h"
//#include "base64.h"

#include "helix.h"

unsigned char *rfc822_binary (void *src,unsigned long srcl,unsigned long *len);

//#include "discid/discid.h"
//#include "discid/discid_private.h"

/*
 * Create a DiscID based on the TOC data found in the DiscId object.
 * The DiscID is placed in the provided string buffer.
 */
 
void computeDiscid(char buf[]) {
	
	SHA_INFO	sha;
	unsigned char	digest[20], *base64;
	unsigned long	size;
	char		tmp[17]; /* for 8 hex digits (16 to avoid trouble) */
	int		i;


	for (i = 0;i <= getTrackCount();i++){
			printf ("%02d: %d\n",i,getTrackStart(i));
	}

	sha_init(&sha);

	sprintf(tmp, "%02X", 1);
	sha_update(&sha, (unsigned char *) tmp, strlen(tmp));

	sprintf(tmp, "%02X", getTrackCount());
	sha_update(&sha, (unsigned char *) tmp, strlen(tmp));

	sprintf(tmp, "%08X", 150+getTrackStart(getTrackCount()));
	sha_update(&sha, (unsigned char *) tmp, strlen(tmp));

	for (i = 0; i < 99; i++) {
		if (i < getTrackCount ()) 
			sprintf(tmp, "%08X", 150+getTrackStart(i));
		else 
			sprintf(tmp, "%08X", 0);
						
		sha_update(&sha, (unsigned char *) tmp, strlen(tmp));
	}

	sha_final(digest, &sha);

	base64 = rfc822_binary(digest, sizeof(digest), &size);

	memcpy(buf, base64, size);
	buf[size] = '\0';

	printf ("ComputeId => %s\n",buf);

	free(base64);
}

