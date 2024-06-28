/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SIB9_H_
#define	_SIB9_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>
#include <INTEGER.h>
#include <BIT_STRING.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ReferenceTimeInfo_r16;

/* SIB9 */
typedef struct SIB9 {
	struct SIB9__timeInfo {
		INTEGER_t	 timeInfoUTC;
		BIT_STRING_t	*dayLightSavingTime;	/* OPTIONAL */
		long	*leapSeconds;	/* OPTIONAL */
		long	*localTimeOffset;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *timeInfo;
	OCTET_STRING_t	*lateNonCriticalExtension;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SIB9__ext1 {
		struct ReferenceTimeInfo_r16	*referenceTimeInfo_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SIB9_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SIB9;
extern asn_SEQUENCE_specifics_t asn_SPC_SIB9_specs_1;
extern asn_TYPE_member_t asn_MBR_SIB9_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ReferenceTimeInfo-r16.h"

#endif	/* _SIB9_H_ */
#include <asn_internal.h>
