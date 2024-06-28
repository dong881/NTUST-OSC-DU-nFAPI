/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RxTxPeriodical_r17_H_
#define	_RxTxPeriodical_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RxTxReportInterval-r17.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RxTxPeriodical_r17__reportAmount_r17 {
	RxTxPeriodical_r17__reportAmount_r17_r1	= 0,
	RxTxPeriodical_r17__reportAmount_r17_infinity	= 1,
	RxTxPeriodical_r17__reportAmount_r17_spare6	= 2,
	RxTxPeriodical_r17__reportAmount_r17_spare5	= 3,
	RxTxPeriodical_r17__reportAmount_r17_spare4	= 4,
	RxTxPeriodical_r17__reportAmount_r17_spare3	= 5,
	RxTxPeriodical_r17__reportAmount_r17_spare2	= 6,
	RxTxPeriodical_r17__reportAmount_r17_spare1	= 7
} e_RxTxPeriodical_r17__reportAmount_r17;

/* RxTxPeriodical-r17 */
typedef struct RxTxPeriodical_r17 {
	RxTxReportInterval_r17_t	*rxTxReportInterval_r17;	/* OPTIONAL */
	long	 reportAmount_r17;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RxTxPeriodical_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_reportAmount_r17_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_RxTxPeriodical_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_RxTxPeriodical_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_RxTxPeriodical_r17_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _RxTxPeriodical_r17_H_ */
#include <asn_internal.h>
