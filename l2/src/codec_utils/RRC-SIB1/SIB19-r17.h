/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SIB19_r17_H_
#define	_SIB19_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <INTEGER.h>
#include "ReferenceLocation-r17.h"
#include <NativeInteger.h>
#include <OCTET_STRING.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct NTN_Config_r17;
struct NTN_NeighCellConfigList_r17;

/* SIB19-r17 */
typedef struct SIB19_r17 {
	struct NTN_Config_r17	*ntn_Config_r17;	/* OPTIONAL */
	INTEGER_t	*t_Service_r17;	/* OPTIONAL */
	ReferenceLocation_r17_t	*referenceLocation_r17;	/* OPTIONAL */
	long	*distanceThresh_r17;	/* OPTIONAL */
	struct NTN_NeighCellConfigList_r17	*ntn_NeighCellConfigList_r17;	/* OPTIONAL */
	OCTET_STRING_t	*lateNonCriticalExtension;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SIB19_r17__ext1 {
		struct NTN_NeighCellConfigList_r17	*ntn_NeighCellConfigListExt_v1720;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SIB19_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SIB19_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_SIB19_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SIB19_r17_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "NTN-Config-r17.h"
#include "NTN-NeighCellConfigList-r17.h"

#endif	/* _SIB19_r17_H_ */
#include <asn_internal.h>
